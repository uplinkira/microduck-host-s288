"""只读 MJCF 静态预检，不执行 MuJoCo、不展开 include/default、不认证物理可行。"""
import hashlib
import math
from pathlib import Path
import re
import xml.etree.ElementTree as ET

from robot_profile import validate_profile, profile_digest

MAX_XML_BYTES = 8 * 1024 * 1024
MAX_XML_NODES = 50000
MAX_ASSET_BYTES = 256 * 1024 * 1024
HASH_CHUNK_BYTES = 1024 * 1024


class _LimitedTreeBuilder(ET.TreeBuilder):
    def __init__(self):
        super().__init__()
        self.count = 0

    def start(self, tag, attrs):
        self.count += 1
        if self.count > MAX_XML_NODES:
            raise ValueError('XML 节点超过 50000 资源上限')
        return super().start(tag, attrs)


def inspect_model(path, profile=None):
    source = Path(path).resolve()
    checks = []
    def add(level, code, message):
        checks.append({'level': level, 'code': code, 'message': message})
    report = {'path': str(source), 'sha256': None,
              'scope': 'root_xml_static_only', 'certified': False, 'training_ready': False,
              'checks': checks, 'joint_names': [], 'mesh_files': [], 'assets': [], 'profile_digest': None}
    def finish():
        report['ok'] = not any(c['level'] == 'error' for c in checks)
        report['status'] = 'failed' if not report['ok'] else 'review_required'
        return report
    if profile is not None:
        try:
            checks.extend(validate_profile(profile))
            report['profile_digest'] = profile_digest(profile)
        except ValueError as exc:
            add('error', 'profile_invalid', str(exc))
    if source.stat().st_size > MAX_XML_BYTES:
        add('error', 'xml_size_limit', 'XML 文件超过 8 MiB 资源上限')
        return finish()
    with source.open('rb') as stream:
        data = stream.read(MAX_XML_BYTES + 1)
    if len(data) > MAX_XML_BYTES:
        add('error', 'xml_size_limit', 'XML 文件超过 8 MiB 资源上限')
        return finish()
    report['sha256'] = hashlib.sha256(data).hexdigest()
    try:
        xml_text = data.decode('utf-8-sig')
    except UnicodeDecodeError:
        add('error', 'xml_encoding', '模型 XML 必须采用 UTF-8 编码')
        return finish()
    declaration = re.match(r'\s*<\?xml\s+[^?]*encoding\s*=\s*[\"\x27]([^\"\x27]+)', xml_text, re.I)
    if declaration and declaration.group(1).lower().replace('_', '-') not in ('utf-8', 'utf8'):
        add('error', 'xml_encoding', 'XML 编码声明必须为 UTF-8')
        return finish()
    if b'<!DOCTYPE' in data.upper() or b'<!ENTITY' in data.upper():
        add('error', 'xml_declaration_unsupported', '不处理 DTD/实体声明')
        return finish()
    try:
        root = ET.fromstring(xml_text, parser=ET.XMLParser(target=_LimitedTreeBuilder()))
    except (ET.ParseError, ValueError) as exc:
        add('error', 'invalid_xml', str(exc))
        return finish()
    if root.tag != 'mujoco':
        add('error', 'not_mjcf', '根元素必须是 mujoco')
        return finish()
    unresolved = any(root.findall('.//' + tag) for tag in ('include', 'default', 'attach', 'replicate'))
    for inc in root.findall('.//include'):
        add('warning', 'include_unresolved', f'未展开 include：{inc.get("file", "未命名")}；节点和资源统计不完整')
    if root.findall('.//default'):
        add('warning', 'default_unresolved', '未解析 default/class 继承，缺失属性不能按默认值认证')
    for tag in ('attach', 'replicate'):
        if root.findall('.//' + tag):
            add('warning', tag + '_unresolved', f'未展开 {tag}；模型节点统计和引用不完整')
    for element in root.iter():
        for field in ('axis', 'range', 'scale'):
            if field not in element.attrib:
                continue
            try:
                numbers = [float(value) for value in element.get(field).split()]
                if not numbers or not all(math.isfinite(value) for value in numbers):
                    raise ValueError()
            except ValueError:
                add('error', 'nonfinite_' + field, f'{element.tag} {element.get("name", "")} 的 {field} 必须为有限数字')
    world = root.find('worldbody')
    if world is None:
        add('warning' if unresolved else 'error', 'missing_worldbody', '根文件未发现 worldbody')
        bodies, joints = [], []
    else:
        bodies = world.findall('.//body')
        joints = world.findall('.//joint')
    names = [j.get('name') for j in joints if j.get('type', 'hinge') != 'free']
    report['joint_names'] = names
    if any(not name for name in names) or len(set(names)) != len(names):
        add('error', 'joint_names_invalid', '运动关节存在缺名或重复名称')
    roots = world.findall('body') if world is not None else []
    free = any(b.find('freejoint') is not None or any(j.get('type') == 'free' for j in b.findall('joint')) for b in roots)
    if not free:
        add('warning' if unresolved else 'error', 'floating_base_missing', '未发现根 body 的自由基座；固定底座不能验证行走')
    compiler = root.find('compiler')
    if compiler is not None:
        for field in ('balanceinertia', 'boundinertia', 'boundmass', 'settotalmass'):
            if field in compiler.attrib:
                add('warning', 'compiler_' + field, f'compiler {field}={compiler.get(field)}；自动调整不能代替原始质量/惯量核验')
    add('warning', 'units_unverified', 'MJCF 长度/质量单位与实物来源需人工确认；不自动换算疑似克数据')
    add('info', 'angle_unit', '角度声明：' + (compiler.get('angle', '未显式声明（MuJoCo 通常默认 degree）') if compiler is not None else '未显式声明（MuJoCo 通常默认 degree）'))
    meshdir = compiler.get('meshdir', '') if compiler is not None else ''
    if compiler is not None and compiler.get('assetdir'):
        meshdir = compiler.get('meshdir', compiler.get('assetdir'))
    hashed = set()
    def check_resource(filename, directory='', is_mesh=False, hash_asset=False):
        # 先检查词法和规范路径；不 stat/open 包目录以外的目标，不联网。
        for component in (filename, directory):
            if re.match(r'^[a-zA-Z][a-zA-Z0-9+.-]*:', component) or component.startswith(('//', '\\\\')):
                add('error', 'resource_uri_or_absolute', f'拒绝 URI/绝对资源路径：{component}')
                return
        resolved = (source.parent / directory / filename).resolve()
        if not resolved.is_relative_to(source.parent):
            add('error', 'resource_outside_package', f'资源越出模型包目录：{filename}')
            return
        if is_mesh:
            report['mesh_files'].append(str(resolved))
        if not resolved.is_file():
            add('warning' if unresolved else 'error', 'mesh_missing' if is_mesh else 'asset_missing', f'资源不存在：{resolved}')
            return
        if not hash_asset or resolved in hashed:
            return
        if resolved.stat().st_size > MAX_ASSET_BYTES:
            add('error', 'asset_size_limit', f'资产超过 256 MiB：{resolved}')
            return
        digest, count = hashlib.sha256(), 0
        with resolved.open('rb') as stream:
            while chunk := stream.read(HASH_CHUNK_BYTES):
                count += len(chunk)
                if count > MAX_ASSET_BYTES:
                    add('error', 'asset_size_limit', f'资产读取期间超过 256 MiB：{resolved}')
                    return
                digest.update(chunk)
        report['assets'].append({'path': str(resolved), 'sha256': digest.hexdigest()})
        hashed.add(resolved)
    for inc in root.findall('.//include'):
        if inc.get('file'):
            check_resource(inc.get('file'))
    for mesh in root.findall('./asset/*'):
        filename = mesh.get('file')
        if not filename:
            if mesh.tag == 'mesh':
                add('warning', 'mesh_inline_unchecked', '未检查内联 mesh 几何')
            continue
        if compiler is not None and compiler.get('strippath') == 'true':
            add('warning', 'mesh_path_semantics', 'strippath 未求值，资源存在性检查需 MuJoCo 编译复核')
        directory = meshdir if mesh.tag == 'mesh' else ''
        if mesh.tag == 'texture' and compiler is not None:
            directory = compiler.get('texturedir', compiler.get('assetdir', ''))
        check_resource(filename, directory, mesh.tag == 'mesh', True)
    explicit_mass = 0.0
    for body in bodies:
        inertial = body.find('inertial')
        label = body.get('name', '未命名 body')
        if inertial is None:
            add('warning', 'inertia_inferred', f'{label} 无显式 inertial；不推断 geom 密度/惯量')
            continue
        try:
            mass = float(inertial.get('mass', 'nan'))
            if not math.isfinite(mass) or mass <= 0:
                raise ValueError()
            explicit_mass += mass
            if mass > 10:
                add('warning', 'mass_magnitude', f'{label} 显式质量 {mass:g} 超过 10；若按 kg 解释量级可疑，不能认定原单位或自动换算')
        except ValueError:
            add('error', 'mass_invalid', f'{label} 显式质量必须为有限正数')
        if inertial.get('diaginertia'):
            try:
                inertia = [float(x) for x in inertial.get('diaginertia').split()]
                if len(inertia) != 3 or any(not math.isfinite(x) or x <= 0 for x in inertia) or 2 * max(inertia) > sum(inertia) + 1e-12 * max(inertia):
                    raise ValueError()
            except ValueError:
                add('error', 'inertia_invalid', f'{label} 主惯量非正、非有限或违反三角条件')
        elif inertial.get('fullinertia'):
            add('warning', 'fullinertia_unchecked', f'{label} 完整惯量张量需编译/特征值检查')
        else:
            add('warning', 'inertia_unspecified', f'{label} 未显式提供惯量')
    report['explicit_mass_sum'] = explicit_mass
    if explicit_mass > 10:
        add('warning', 'mass_sum_magnitude', f'显式质量和 {explicit_mass:g} 超过 10；需核对小型鸭模型的质量单位和来源')
    report['body_count'] = len(bodies)
    actuator = root.find('actuator')
    report['actuator_count'] = len(actuator) if actuator is not None else 0
    if not report['actuator_count']:
        add('warning', 'actuators_missing', '根文件无执行器；若由训练代码注入，须验证实际生成模型')
    add('warning', 'physics_unverified', '未验证接触、站姿稳定、质量单位、碰撞与执行器动态；报告不代表可训练或可部署')
    if report['profile_digest'] is not None:
        for joint in profile['joints']:
            name = joint['model_joint']
            if name and name not in names:
                add('warning' if unresolved else 'error', 'profile_joint_missing', f'构型关节未在根模型找到：{name}')
    return finish()
