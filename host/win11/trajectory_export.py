"""导出离线关节参考；主机草案可导出，模型角必须有明确构型与标定。

没有根部/身体状态，任何输出都不标记为 mjlab training-ready。
CSV 每行携带坐标与构型摘要；完整构型、段落及设计参数请使用 JSON。
"""
import copy
import csv
import json
import math
import os
from pathlib import Path
import tempfile

from trajectory import SCHEMA, MAX_SAMPLES, MAX_DURATION_S
from robot_profile import validate_profile, profile_digest, write_json


def _validate_plan(plan):
    if not isinstance(plan, dict) or plan.get('schema') != SCHEMA:
        raise ValueError('不是支持的离线参考计划')
    if plan.get('coordinate_frame') != 'host_zero_relative':
        raise ValueError('输入必须是主机零位相对角度，不能重复转换模型坐标')
    if plan.get('units') != dict(position='deg', velocity='deg/s', acceleration='deg/s2', time='s'):
        raise ValueError('参考计划单位必须明确为 deg、deg/s、deg/s2、s')
    ids = plan.get('joint_ids')
    if (not isinstance(ids, list) or not 1 <= len(ids) <= 15
            or any(type(mid) is not int or not 0 <= mid <= 14 for mid in ids)
            or len(set(ids)) != len(ids)):
        raise ValueError('参考计划 joint_ids 无效')
    samples = plan.get('samples')
    if not isinstance(samples, list) or not 1 <= len(samples) <= MAX_SAMPLES:
        raise ValueError('参考采样为空或超过资源上限')
    previous = -1.0
    for sample in samples:
        if not isinstance(sample, dict):
            raise ValueError('采样帧必须是对象')
        t = sample.get('t_s')
        if (type(t) not in (int, float) or not math.isfinite(t)
                or t <= previous or not 0 <= t <= MAX_DURATION_S):
            raise ValueError('采样时间必须有限、严格递增且在资源边界内')
        previous = t
        for key in ('q_deg', 'dq_deg_s', 'ddq_deg_s2'):
            vector = sample.get(key)
            if (not isinstance(vector, list) or len(vector) != len(ids)
                    or any(type(x) not in (int, float) or not math.isfinite(x) for x in vector)):
                raise ValueError(f'{key} 维度或数值无效')
    if samples[0]['t_s'] != 0 or plan.get('duration_s') != previous:
        raise ValueError('采样起止时间与计划时长不一致')
    # 也检查附加元数据的可序列化性，禁止 NaN 混入输出。
    json.dumps(plan, ensure_ascii=False, allow_nan=False)
    return ids


def _model_mapping(profile, ids):
    checks = validate_profile(profile)
    errors = [check['message'] for check in checks if check['level'] == 'error']
    if errors:
        raise ValueError('模型坐标导出需要完整已确认构型：' + '；'.join(errors[:4]))
    active = [joint for joint in profile['joints'] if joint['role'] != 'passive']
    if len({joint['bus'] for joint in active}) != 1:
        raise ValueError('当前参考仅按 ID 寻址，模型导出只支持单总线构型')
    mapping = {joint['motor_id']: joint for joint in active}
    if len(mapping) != len(active) or any(mid not in mapping for mid in ids):
        raise ValueError('构型无法唯一映射全部参考 ID')
    for mid in ids:
        joint = mapping[mid]
        if (joint['verified'] is not True or joint['direction'] not in (-1, 1)
                or joint['q_home_rad'] is None or not joint['model_joint']):
            raise ValueError(f'ID {mid} 缺少确认的模型映射、方向或 q_home')
    return [mapping[mid] for mid in ids]


def export_reference(path, plan, profile=None, model_coordinates=False):
    """根据 .json/.csv 原子导出，返回导出摘要；失败不覆盖旧目标。

    模型位置 = q_home_rad + direction × radians(host_deg)。速度/加速度
    只乘方向与单位换算，不加零偏。输出顺序仍为 joint_ids，绝不偷换为策略索引。
    """
    target = Path(path)
    if target.suffix.lower() not in ('.json', '.csv'):
        raise ValueError('参考轨迹只支持 .json 或 .csv')
    if type(model_coordinates) is not bool:
        raise ValueError('model_coordinates 必须为布尔值')
    ids = _validate_plan(plan)
    digest = None
    checks = []
    if profile is not None:
        checks = validate_profile(profile)
        digest = profile_digest(profile)
    if model_coordinates and profile is None:
        raise ValueError('模型坐标导出必须提供已确认构型')
    mapping = _model_mapping(profile, ids) if model_coordinates else None
    result = copy.deepcopy(plan)
    result['schema'] = 'xianduck_joint_reference_v1'
    result['source_plan_schema'] = SCHEMA
    result['profile_digest'] = digest
    result['profile'] = copy.deepcopy(profile)
    result['profile_checks'] = checks
    result['training_ready'] = False
    result['hardware_validated'] = False
    result['data_kind'] = 'planned_joint_reference'
    result['missing_training_fields'] = ['root_pose', 'body_positions', 'body_quaternions',
                                         'body_linear_velocities', 'body_angular_velocities']
    result['joint_names'] = [joint['model_joint'] for joint in mapping] if mapping else [f'id_{mid}' for mid in ids]
    result['joint_order'] = 'joint_ids_not_policy_indices'
    result['coordinate_frame'] = 'model_joint_coordinates' if mapping else 'host_zero_relative'
    position, velocity, acceleration = ('q_rad', 'dq_rad_s', 'ddq_rad_s2') if mapping else ('q_deg', 'dq_deg_s', 'ddq_deg_s2')
    if mapping:
        result['units'] = dict(position='rad', velocity='rad/s', acceleration='rad/s2', time='s')
        result['samples'] = [dict(t_s=sample['t_s'],
            q_rad=[joint['q_home_rad'] + joint['direction'] * math.radians(x)
                   for joint, x in zip(mapping, sample['q_deg'])],
            dq_rad_s=[joint['direction'] * math.radians(x)
                      for joint, x in zip(mapping, sample['dq_deg_s'])],
            ddq_rad_s2=[joint['direction'] * math.radians(x)
                        for joint, x in zip(mapping, sample['ddq_deg_s2'])]) for sample in plan['samples']]
        result['conversion'] = 'q_model_rad=q_home_rad+direction*host_deg*pi/180'
    # JSON 和 CSV 都检查整个结果，不能因 CSV 跳过元数据序列化而漏掉非法值。
    json.dumps(result, ensure_ascii=False, allow_nan=False)
    # 度单位的设计参数保留明确字段后缀；转换不会伪造动力学或根部状态。
    if target.suffix.lower() == '.json':
        write_json(target, result)
    else:
        target.parent.mkdir(parents=True, exist_ok=True)
        fd, temporary = tempfile.mkstemp(prefix=target.name + '.', suffix='.tmp', dir=target.parent)
        try:
            with os.fdopen(fd, 'w', encoding='utf-8-sig', newline='') as stream:
                writer = csv.writer(stream)
                fields = [position, velocity, acceleration]
                writer.writerow(['t_s', 'coordinate_frame', 'profile_digest', 'training_ready', 'data_kind'] +
                                [f'{name}__{field}' for field in fields for name in result['joint_names']])
                for sample in result['samples']:
                    writer.writerow([sample['t_s'], result['coordinate_frame'], digest or '', 'false',
                                     'planned_joint_reference'] +
                                    [x for field in fields for x in sample[field]])
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temporary, target)
        finally:
            if os.path.exists(temporary):
                os.unlink(temporary)
    return dict(path=str(target), samples=len(result['samples']),
                coordinate_frame=result['coordinate_frame'], profile_digest=digest,
                training_ready=False)
