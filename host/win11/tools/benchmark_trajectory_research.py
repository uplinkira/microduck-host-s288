"""Ruckig 研究基准：只做内存计算，不导入控制器、不访问串口。

研究依赖单独安装到 build/research_ruckig_0194，不改变生产依赖。
加速度和 jerk 是人工测试值，不是 S288 的实测安全值。
"""
import argparse
import json
import math
from pathlib import Path
import statistics
import sys
import time


def main():
    if hasattr(sys.stdout, 'reconfigure'):
        sys.stdout.reconfigure(encoding='utf-8')
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--samples', type=int, default=2001)
    args = parser.parse_args()
    if args.samples < 3 or args.samples > 100001:
        parser.error('samples 须在 3 到 100001 之间')
    dependency = Path(__file__).resolve().parents[1] / 'build/research_ruckig_0194'
    if not dependency.is_dir():
        parser.error('请按研究文档安装隔离的 ruckig==0.19.4')
    sys.path.insert(0, str(dependency))
    from ruckig import Ruckig, InputParameter, Trajectory, Result, Synchronization

    cases = []
    for delta in ([0.0]*14, [.001]*14, [.2, .1, -.1]+[0.0]*11, [2., -1.]+[.3]*12):
        inp = InputParameter(14)
        inp.current_position = [0.0]*14
        inp.current_velocity = [0.0]*14
        inp.current_acceleration = [0.0]*14
        inp.target_position = delta
        inp.target_velocity = [0.0]*14
        inp.target_acceleration = [0.0]*14
        inp.max_velocity = [math.radians(12.5)]*14
        inp.max_acceleration = [math.radians(25)]*14
        inp.max_jerk = [math.radians(100)]*14
        inp.synchronization = Synchronization.Time
        # 不传 intermediate_positions；社区版多路点会转云端，不属于此基准。
        otg, trajectory = Ruckig(14), Trajectory(14)
        times = []
        for _ in range(100):
            start = time.perf_counter()
            result = otg.calculate(inp, trajectory)
            times.append((time.perf_counter()-start)*1e6)
            if result != Result.Working:
                raise RuntimeError(f'求解失败：{result}')
        states = [trajectory.at_time(trajectory.duration*i/(args.samples-1))
                  for i in range(args.samples)]
        error = max(abs(states[-1][0][j]-delta[j]) for j in range(14))
        vmax = max(abs(v) for _, velocities, _ in states for v in velocities)
        amax = max(abs(a) for _, _, accelerations in states for a in accelerations)
        dt = trajectory.duration/(args.samples-1)
        jmax = max(abs(states[i+1][2][j]-states[i][2][j])/dt
                   for i in range(args.samples-1) for j in range(14)) if dt else 0.0
        endpoint = max(abs(x) for state in (states[0], states[-1])
                       for derivative in state[1:] for x in derivative)
        if not (error < 1e-9 and endpoint < 1e-9
                and vmax <= inp.max_velocity[0]+1e-9
                and amax <= inp.max_acceleration[0]+1e-9
                and jmax <= inp.max_jerk[0]+1e-7):
            raise AssertionError('采样或端点约束检查失败')
        cases.append(dict(max_move_rad=max(map(abs, delta)), duration_s=trajectory.duration,
                          vmax_rad_s=vmax, amax_rad_s2=amax, sampled_jmax_rad_s3=jmax,
                          end_error_rad=error, endpoint_derivative_error=endpoint,
                          median_us=statistics.median(times), max_us=max(times)))
    print(json.dumps(dict(backend='ruckig', version='0.19.4', dof=14, samples=args.samples,
                          physical_parameters='人工研究输入，不是 S288 安全包络',
                          continuous_time_extrema_proven=False, hardware_tested=False,
                          cases=cases), ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
