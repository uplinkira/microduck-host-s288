"""离线构型契约。未知标定保留 None；verified 仅指映射确认，不是实机认证。"""
import copy
import hashlib
import json
import math
import os
from pathlib import Path
import tempfile


UNITS = {'position': 'rad', 'velocity': 'rad/s', 'length': 'm', 'mass': 'kg', 'time': 's'}
MAX_JOINTS = 128
MAX_JSON_BYTES = 8 * 1024 * 1024


def _number(value):
    try:
        return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)
    except OverflowError:
        return False


def make_profile(joints):
    """创建 schema v1 草稿；调用者仍需 validate_profile 查看待确认项。"""
    result = {'schema_version': 1, 'name': 'xianduck_s288', 'units': dict(UNITS),
              'joints': copy.deepcopy(list(joints))}
    validate_profile(result)
    return result


def draft_from_hardware(specs):
    """接受 {ID: spec} 或 spec 列表；硬件语义名不能推断为模型关节名。"""
    items = specs.items() if isinstance(specs, dict) else [(None, s) for s in specs]
    joints = []
    for key, spec in items:
        mid = spec.get('motor_id', spec.get('id', key))
        semantic = spec.get('semantic', spec.get('joint'))
        role = spec.get('role', 'auxiliary' if mid == 14 or semantic == 'mouth' else 'policy')
        joints.append({'motor_id': mid, 'semantic': semantic, 'model_joint': None,
                       'bus': None, 'direction': None, 'q_home_rad': None,
                       'role': role, 'policy_index': None, 'verified': False})
    return make_profile(joints)


def validate_profile(profile):
    """结构/非法数值抛 ValueError；缺失标定返回 error 检查，不补默认零。"""
    if not isinstance(profile, dict) or type(profile.get('schema_version')) is not int or profile['schema_version'] != 1:
        raise ValueError('不支持的构型 schema_version，要求整数 1')
    if not isinstance(profile.get('name'), str) or not profile['name'].strip() or len(profile['name']) > 128:
        raise ValueError('构型 name 必须为非空字符串且不超过 128 个字符')
    if profile.get('units') != UNITS:
        raise ValueError('构型单位必须是 rad、rad/s、m、kg、s')
    joints = profile.get('joints')
    if not isinstance(joints, list) or not 1 <= len(joints) <= MAX_JOINTS:
        raise ValueError(f'joints 必须是 1–{MAX_JOINTS} 项的列表')
    checks, seen, models, semantics, indices = [], set(), set(), set(), []
    required = {'motor_id', 'semantic', 'model_joint', 'bus', 'direction', 'q_home_rad', 'role', 'policy_index', 'verified'}
    def pending(code, message):
        checks.append({'level': 'error', 'code': code, 'message': message})
    for n, j in enumerate(joints):
        if not isinstance(j, dict) or not required <= j.keys():
            raise ValueError(f'关节 {n} 缺少构型字段')
        role = j['role']
        if role not in ('policy', 'auxiliary', 'passive') or type(j['verified']) is not bool:
            raise ValueError(f'关节 {n} role/verified 非法')
        for field, names in [('semantic', semantics), ('model_joint', models)]:
            value = j[field]
            if value is None or value == '':
                pending('missing_' + field, f'关节 {n} 缺少 {field}')
            elif not isinstance(value, str) or not value.strip() or value in names:
                raise ValueError(f'关节 {n} 的 {field} 类型错误或重复')
            else:
                names.add(value)
        mid, bus = j['motor_id'], j['bus']
        if bus is not None and (not isinstance(bus, str) or not bus.strip()):
            raise ValueError(f'关节 {n} bus 必须为非空字符串或 null')
        if role == 'passive':
            if mid is not None or j['policy_index'] is not None:
                raise ValueError('被动关节不能设置 motor_id 或 policy_index')
        else:
            if type(mid) is not int or not 0 <= mid <= 14:
                raise ValueError('S288 单台 motor_id 必须为整数 0–14；15 是广播')
            address = (bus, mid)
            if address in seen:
                raise ValueError(f'重复总线地址 {address}')
            seen.add(address)
            if bus is None:
                pending('missing_bus', f'ID {mid} 总线待确认')
            if j['direction'] is None:
                pending('missing_direction', f'ID {mid} 方向待确认')
            if j['q_home_rad'] is None:
                pending('missing_q_home', f'ID {mid} 模型站姿待确认')
        if j['direction'] is not None and (type(j['direction']) is not int or j['direction'] not in (-1, 1)):
            raise ValueError('direction 只能为 -1、1 或 null')
        if j['q_home_rad'] is not None and not _number(j['q_home_rad']):
            raise ValueError('q_home_rad 必须为有限数或 null')
        index = j['policy_index']
        if role == 'policy':
            if index is None:
                pending('missing_policy_index', f'关节 {n} 策略顺序待确认')
            elif type(index) is not int or index < 0 or index in indices:
                raise ValueError('policy_index 必须为唯一非负整数')
            else:
                indices.append(index)
        elif index is not None:
            raise ValueError('非策略关节不能设置 policy_index')
        if not j['verified']:
            pending('unverified_mapping', f'关节 {n} 映射未确认')
    policy_count = sum(j['role'] == 'policy' for j in joints)
    if len(indices) == policy_count and sorted(indices) != list(range(policy_count)):
        raise ValueError('完整 policy_index 必须从 0 连续编号')
    return checks


def profile_digest(profile):
    """保守哈希所有配置字段；任何改变均需要重新审查原验证证据。"""
    validate_profile(profile)
    payload = json.dumps(profile, ensure_ascii=False, sort_keys=True, separators=(',', ':'), allow_nan=False)
    return hashlib.sha256(payload.encode('utf-8')).hexdigest()


def diff_profiles(old, new):
    """返回递归字段差异；不修改原档案，不自动迁移验证状态。"""
    before, after = profile_digest(old), profile_digest(new)
    changes = []
    def walk(a, b, path):
        if isinstance(a, dict) and isinstance(b, dict):
            for key in sorted(a.keys() | b.keys()):
                if key not in a or key not in b:
                    changes.append({'path': path + key, 'old': a.get(key), 'new': b.get(key)})
                else:
                    walk(a[key], b[key], path + key + '.')
        elif a != b:
            changes.append({'path': path.rstrip('.'), 'old': copy.deepcopy(a), 'new': copy.deepcopy(b)})
    walk(old, new, '')
    return {'changed': before != after, 'validation_invalidated': before != after,
            'old_digest': before, 'new_digest': after, 'changes': changes}


def read_json(path):
    def reject(value):
        raise ValueError('JSON 不允许非有限值：' + value)
    def unique(pairs):
        result = {}
        for key, value in pairs:
            if key in result:
                raise ValueError('JSON 重复字段：' + key)
            result[key] = value
        return result
    source = Path(path)
    if source.stat().st_size > MAX_JSON_BYTES:
        raise ValueError('JSON 文件超过 8 MiB 资源上限')
    with source.open('rb') as stream:
        payload = stream.read(MAX_JSON_BYTES + 1)
    if len(payload) > MAX_JSON_BYTES:
        raise ValueError('JSON 文件超过 8 MiB 资源上限')
    return json.loads(payload.decode('utf-8-sig'), parse_constant=reject, object_pairs_hook=unique)


def write_json(path, value):
    """UTF-8 原子写入；序列化失败不破坏已有文件。"""
    payload = json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + '\n'
    if len(payload.encode('utf-8')) > MAX_JSON_BYTES:
        raise ValueError('JSON 内容超过 8 MiB 资源上限')
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary = tempfile.mkstemp(prefix=target.name + '.', suffix='.tmp', dir=target.parent)
    try:
        with os.fdopen(fd, 'w', encoding='utf-8', newline='\n') as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, target)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)
