"""离线 Ruckig 状态到状态参考轨迹；不导入控制器、不打开串口。

每个动作是精确停止点，速度为上限，暂停计入参考时间轴。加速度和
跃度是设计输入，不是实测安全认证；有限跃度不代表跃度连续。
"""
import math
from collections.abc import Mapping


SCHEMA = 'xianduck_reference_plan_v1'
RUCKIG_VERSION = '0.19.4'
MAX_POSES = 1000
MAX_SAMPLES = 200000
MAX_DURATION_S = 3600.0
MAX_SAMPLE_HZ = 1000.0
# 算法数值/资源边界，不是关节角度软限位，不会裁剪输入。
MAX_NUMERIC = 100000000.0


class PlanningCancelled(RuntimeError):
    """用户取消离线计算；没有可执行的部分轨迹返回。"""


def _number(value, label, *, positive=False, nonnegative=False, maximum=MAX_NUMERIC):
    if isinstance(value, bool):
        raise ValueError(f'{label} 不能是布尔值')
    try:
        result = float(value)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(f'{label} 必须是有限数值') from exc
    if not math.isfinite(result) or abs(result) > maximum:
        raise ValueError(f'{label} 超出有限数值计算范围')
    if positive and result <= 0 or nonnegative and result < 0:
        raise ValueError(f'{label} 必须{ "大于" if positive else "不小于" }零')
    return result


def _angles(values, label):
    if not isinstance(values, Mapping) or not values or len(values) > 15:
        raise ValueError(f'{label} 需要 1–15 个明确 ID 的角度')
    result = {}
    for raw, angle in values.items():
        if isinstance(raw, bool) or not isinstance(raw, (int, str)):
            raise ValueError(f'{label} 的 ID 必须为 0–14 整数，15 为广播')
        try:
            mid = int(raw)
        except (ValueError, TypeError) as exc:
            raise ValueError(f'{label} 的 ID 无效') from exc
        if not 0 <= mid <= 14 or isinstance(raw, str) and str(mid) != raw:
            raise ValueError(f'{label} 的 ID 必须为 0–14，且不能有别名')
        if mid in result:
            raise ValueError(f'{label} 存在重复 ID {mid}')
        result[mid] = _number(angle, f'{label} ID {mid} 角度')
    return result


def _check_cancel(cancel):
    if cancel is not None:
        stopped = cancel() if callable(cancel) else cancel.is_set()
        if stopped:
            raise PlanningCancelled('已取消离线轨迹生成')


def _backend():
    try:
        import ruckig
    except ImportError as exc:
        raise RuntimeError('缺少本地轨迹后端，请安装固定依赖 ruckig==0.19.4') from exc
    version = getattr(ruckig, '__version__', None)
    if version != RUCKIG_VERSION:
        raise RuntimeError(f'需要 ruckig=={RUCKIG_VERSION}，当前为 {version or "未知版本"}')
    return ruckig


def plan_poses(poses, start_angles=None, max_acceleration=25, max_jerk=100,
               sample_hz=50, cancel=None):
    """返回 JSON 可序列化的、按 ID 排序的角度参考计划。

    poses 使用现有动作库的 name/angles/speed/dwell 字段。start_angles=None
    时首动作定义初始位置，只保留该动作暂停；不会猜测零位。每帧 ID 集合
    必须一致。采样包含每段精确末端，因而末端附近可能不是等间隔。
    """
    _check_cancel(cancel)
    if not isinstance(poses, (list, tuple)) or not 1 <= len(poses) <= MAX_POSES:
        raise ValueError(f'动作数必须为 1–{MAX_POSES}')
    acceleration = _number(max_acceleration, '加速度', positive=True)
    jerk = _number(max_jerk, '跃度', positive=True)
    hz = _number(sample_hz, '采样频率', positive=True, maximum=MAX_SAMPLE_HZ)
    if hz < 1:
        raise ValueError('采样频率必须为 1–1000 Hz')
    clean = []
    ids = None
    for index, pose in enumerate(poses):
        _check_cancel(cancel)
        if not isinstance(pose, Mapping):
            raise ValueError(f'动作 {index + 1} 必须是对象')
        if 'intermediate_positions' in pose or pose.get('mode', 'stop') != 'stop':
            raise ValueError('仅支持本地精确停点，不支持云端途经点或 blend')
        if 'speed' not in pose and 'trans' in pose:
            raise ValueError('旧过渡时长需先在动作库明确迁移为速度')
        angles = _angles(pose.get('angles'), f'动作 {index + 1}')
        if ids is None:
            ids = sorted(angles)
        elif set(ids) != set(angles):
            raise ValueError(f'动作 {index + 1} 的 ID 集合不一致，缺失值不能补零')
        speed = _number(pose.get('speed', 12.5), '速度上限', positive=True)
        dwell = _number(pose.get('dwell', 1), '结束暂停', nonnegative=True,
                        maximum=MAX_DURATION_S)
        name = pose.get('name', f'动作 {index + 1}')
        if not isinstance(name, str) or len(name) > 512:
            raise ValueError('动作名称必须为不超过 512 字的文本')
        clean.append(dict(name=name, angles=angles, speed=speed, dwell=dwell))
    initial = clean[0]['angles'] if start_angles is None else _angles(start_angles, '起点')
    if set(initial) != set(ids):
        raise ValueError('起点和动作 ID 集合必须相同')
    backend = _backend()
    count = len(ids)
    solver = backend.Ruckig(count)
    zeros = [0.0] * count
    samples = []
    segments = []
    time_s = 0.0
    current = [initial[mid] for mid in ids]

    def append_sample(t, q, dq, ddq):
        _check_cancel(cancel)
        if len(samples) >= MAX_SAMPLES:
            raise ValueError(f'参考采样超过 {MAX_SAMPLES} 帧，请降低频率或缩短动作')
        values = [t] + list(q) + list(dq) + list(ddq)
        if not all(math.isfinite(x) for x in values):
            raise RuntimeError('求解器输出非有限值')
        if samples and t <= samples[-1]['t_s']:
            raise RuntimeError('轨迹时间精度不足，不能生成严格递增时间轴')
        samples.append(dict(t_s=float(t), q_deg=list(q), dq_deg_s=list(dq),
                            ddq_deg_s2=list(ddq)))

    def reserve_interval(start, duration):
        end = start + duration
        if not math.isfinite(duration) or duration < 0 or end > MAX_DURATION_S:
            raise ValueError(f'总轨迹时长超过 {MAX_DURATION_S:g}s 或求解时长无效')
        if duration > 0 and end <= start:
            raise ValueError('动作时长小于当前时间轴数值精度')
        if len(samples) + math.ceil(duration * hz) > MAX_SAMPLES:
            raise ValueError(f'参考采样超过 {MAX_SAMPLES} 帧，请降低频率或缩短动作')
        return end

    def sample_interval(start, duration, state_at, final_q):
        end = reserve_interval(start, duration)
        if duration == 0:
            return end
        # i/hz 避免累计 dt 误差；末端另采样一次并使用求解已验证的精确目标。
        for i in range(1, math.ceil(duration * hz)):
            local = i / hz
            if local >= duration:
                break
            q, dq, ddq = state_at(local)
            append_sample(start + local, q, dq, ddq)
        append_sample(end, final_q, zeros, zeros)
        return end

    append_sample(0.0, current, zeros, zeros)
    for index, pose in enumerate(clean):
        _check_cancel(cancel)
        target = [pose['angles'][mid] for mid in ids]
        start = time_s
        duration = 0.0
        is_initial = index == 0 and start_angles is None
        if not is_initial and target != current:
            inp = backend.InputParameter(count)
            inp.current_position = [math.radians(q) for q in current]
            inp.target_position = [math.radians(q) for q in target]
            inp.current_velocity = zeros
            inp.current_acceleration = zeros
            inp.target_velocity = zeros
            inp.target_acceleration = zeros
            inp.max_velocity = [math.radians(pose['speed'])] * count
            inp.max_acceleration = [math.radians(acceleration)] * count
            inp.max_jerk = [math.radians(jerk)] * count
            inp.synchronization = backend.Synchronization.Time
            # 绝不设置 intermediate_positions：社区版会调用云 API。
            trajectory = backend.Trajectory(count)
            try:
                if not inp.validate():
                    raise ValueError('Ruckig 输入验证失败')
                result = solver.calculate(inp, trajectory)
                _check_cancel(cancel)
                if result not in (backend.Result.Working, backend.Result.Finished):
                    raise RuntimeError(f'Ruckig 求解失败：{result}')
                duration = trajectory.duration
                reserve_interval(start, duration)
                q_end, v_end, a_end = trajectory.at_time(duration)
                if (max(abs(math.degrees(q) - goal) for q, goal in zip(q_end, target)) > 1e-5
                        or max(abs(x) for x in v_end + a_end) > 1e-7):
                    raise RuntimeError('Ruckig 未达到精确停点边界')

                def state_at(t):
                    return tuple([math.degrees(x) for x in vector]
                                 for vector in trajectory.at_time(t))

                time_s = sample_interval(start, duration, state_at, target)
            except (PlanningCancelled, ValueError, RuntimeError):
                raise
            except Exception as exc:
                raise RuntimeError(f'动作 {index + 1} 本地求解失败：{exc}') from exc
        motion_end = time_s
        time_s = sample_interval(motion_end, pose['dwell'],
                                lambda _t: (target, zeros, zeros), target)
        segments.append(dict(name=pose['name'], start_s=start, motion_end_s=motion_end,
                             end_s=time_s, motion_duration_s=duration,
                             pause_s=pose['dwell'], speed_deg_s=pose['speed'], mode='stop',
                             initial_pose=is_initial))
        current = target
    _check_cancel(cancel)
    return dict(schema=SCHEMA, joint_ids=ids, samples=samples, segments=segments,
                duration_s=time_s, sample_hz=hz, sampling='interval_grid_plus_exact_boundaries',
                coordinate_frame='host_zero_relative', units=dict(position='deg', velocity='deg/s',
                acceleration='deg/s2', time='s'),
                start_source='first_pose' if start_angles is None else 'explicit_angles',
                limits=dict(max_acceleration_deg_s2=acceleration, max_jerk_deg_s3=jerk,
                            speed_semantics='per_segment_peak_upper_bound',
                            hardware_certified=False, position_soft_limits=False),
                backend=dict(name='ruckig', version=RUCKIG_VERSION, local=True,
                             cloud=False, synchronization='time', mode='state_to_state_stop'),
                training_ready=False, hardware_validated=False,
                warnings=['离线设计轨迹，不发送电机；加速度和跃度未经实机认证',
                          '仅关节参考；缺少根部与身体轨迹，不是 mjlab 训练就绪文件',
                          '包含精确段边界，时间间隔不保证完全均匀'])
