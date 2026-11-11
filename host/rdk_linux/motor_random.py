#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
电机随机运转程序(J288/S288 总线)
  - 候选电机: ID 7 / 8 / 14,每轮随机选一台运转
  - ID 14 嘴部: 顺时针向上闭合 / 逆时针向下张开,顺时针随机 20-60°,再逆时针随机 20-60°(两次角度独立抽取)
  - ID 8  摇头: 顺时针向右 / 逆时针向左,顺时针随机 10-30°,再逆时针随机 10-30°
  - ID 7  仰头: 顺时针向上后仰 / 逆时针向下,顺时针随机 3-8°,再逆时针随机 3-8°(幅度最小)
  - 每轮动作结束后先把电机位置恢复到本轮转动前的状态(q0),再间隔 2 秒进入下一轮
  - 程序总时长 1 分钟(60 秒后不再开新轮,当前轮做完即收尾)
  - 转速四档: fast=100°/s / medium=50°/s / slow=25°/s / slowest=12.5°/s(默认最慢)
安全策略(与 motor_sweep.py 一致):
  - timeout 控制位恒 0;Kp=10/Kd=0.5,50ms/步平滑步进
  - MError!=0 或壳温>=85°C: 立即零力矩停机并把该电机剔除出后续轮次
  - 堵转保护: 运动中连续 3 个采样位置误差 >12° 判定机械阻挡,立即卸力剔除(电机14顶限位教训)
  - 连续 3 次无反馈: 视为通信丢失,剔除该电机
  - 温度/电压/扭矩告警做 10 秒限流,避免刷屏
"""
import sys, os, time, math, random, argparse, statistics

if hasattr(sys.stdout, 'reconfigure'):          # 打包成无控制台 exe 时 stdout 是 None
    sys.stdout.reconfigure(encoding='utf-8', errors='replace')
_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)
sys.path.insert(0, os.path.join(os.path.dirname(_HERE), 'common'))   # motor_sweep 在 host/common/
from motor_sweep import Bus, MotorTest, TEMP_ABORT, TEMP_WARN, VOLT_WARN, TORQUE_WARN

# 顺时针/逆时针随机角度范围(度)。顺时针=负方向,逆时针=正方向
ANGLES = {
    7: (3.0, 8.0),     # 仰头: 顺时针向上后仰,逆时针向下。颈部幅度最小
    8: (10.0, 30.0),   # 摇头: 顺时针向右,逆时针向左
    # 嘴部: 顺时针向上闭合,逆时针向下张开。
    # 2026-09-23 实测: 从 347.98° 顺时针闭合到 334.91° 即顶死(约 13°),扭矩饱和在 1.0N·m。
    # 原值 (20,60) 每轮必然硬顶限位,已收窄。真实行程待用 servo_ui.py 逐向标定。
    14: (5.0, 10.0),
}
KP, KD = 10.0, 0.5
DT = 0.05                 # 每步间隔 s
SPEED_TIERS = {'fast': 36, 'medium': 72, 'slow': 144, 'slowest': 288}  # 每180°步数 → 100/50/25/12.5 °/s
STEPS_PER_180 = SPEED_TIERS['slowest']   # 默认最慢档
STALL_DEG = 12.0          # 堵转判定: 位置误差阈值(度)。正常运动中误差只有 0.6~4.5°,12° 有充足余量
STALL_SPD = 1.0           # 堵转判定: 输出端速度阈值(°/s),低于此视为没在动
STALL_SPD_N = 4           # 指令在动但实测不动,连续多少次判堵转(4×50ms=0.2s)
STALL_N = 3               # 连续超阈值采样数
SETTLE = 0.4              # 每个动作终点保持 s
PAUSE = 2.0               # 轮间隔 s
DURATION = 60.0           # 总时长 s


class Runner:
    def __init__(self, bus, mt):
        self.bus = bus
        self.mt = mt
        self.warn_at = {}
        self.comm_fail = 0
        self.stall_hits = 0
        self.slow_hits = 0
        self.stalled = False

    def warn_once(self, key, msg, cooldown=10.0):
        now = time.time()
        if now - self.warn_at.get(key, -1e9) > cooldown:
            self.warn_at[key] = now
            print(f'  ⚠ {msg}', flush=True)

    def check(self, mid, fb):
        """返回 False 表示必须中止该电机"""
        if fb['MError'] != 0:
            print(f'  ✗ 电机{mid} 报错 MError={fb["MError"]},停机剔除', flush=True)
            return False
        if fb['Temp'] >= TEMP_ABORT:
            print(f'  ✗ 电机{mid} 壳温 {fb["Temp"]}°C ≥ {TEMP_ABORT}°C,停机剔除', flush=True)
            return False
        if fb['Temp'] >= TEMP_WARN:
            self.warn_once((mid, 'temp'), f'电机{mid} 壳温 {fb["Temp"]}°C 偏高')
        if fb['vol'] < VOLT_WARN:
            self.warn_once((mid, 'volt'), f'电机{mid} 电压 {fb["vol"]:.1f}V 偏低')
        if abs(fb['outputTor']) > TORQUE_WARN:
            self.warn_once((mid, 'tor'), f'电机{mid} 扭矩 {fb["outputTor"]:.2f}N·m 偏大,可能卡滞')
        return True

    def steps_for(self, deg):
        return max(4, int(round(STEPS_PER_180 * abs(deg) / 180.0)))

    def move(self, mid, q_from, q_to, phase):
        """平滑步进 q_from→q_to(输出端弧度),失败返回 None"""
        n = self.steps_for(math.degrees(q_to - q_from))
        # 前馈轨迹速度: 不发的话 Kd×(0−实际速度) 会在整个运动过程中反向制动,显著加大跟随滞后
        v = (q_to - q_from) / n / DT
        for i in range(1, n + 1):
            q = q_from + (q_to - q_from) * i / n
            fb = self.bus.transact(mid, 1, 0.0, v, q, KP, KD)
            if fb is None:
                self.comm_fail += 1
                if self.comm_fail >= 3:
                    print(f'  ✗ 电机{mid} 连续 {self.comm_fail} 次无反馈,通信丢失剔除', flush=True)
                    return None
                continue
            self.comm_fail = 0
            self.mt.log(mid, phase, q, fb)
            if not self.check(mid, fb):
                return None
            if abs(math.degrees(fb['outputPos'] - q)) > STALL_DEG:
                self.stall_hits += 1
                if self.stall_hits >= STALL_N:
                    self.stalled = True
                    print(f'  ✗ 电机{mid} {phase}: 连续{STALL_N}次误差>{STALL_DEG}°,判定机械阻挡,卸力剔除', flush=True)
                    return None
            else:
                self.stall_hits = 0
            # 指令在动但实测速度≈0 → 机械顶死。比位置误差阈值早得多就能抓住:
            # 实测 ID14 顶限位时第 30 步速度已归零,而误差要到第 38 步才接近 12°
            if abs(math.degrees(v)) > 2.0 and abs(math.degrees(fb['outputSpd'])) < STALL_SPD:
                self.slow_hits += 1
                if self.slow_hits >= STALL_SPD_N:
                    self.stalled = True
                    print(f'  ✗ 电机{mid} {phase}: 指令 {math.degrees(v):+.1f}°/s 但实测静止 '
                          f'{STALL_SPD_N} 次,扭矩 {fb["outputTor"]:+.2f}N·m,判机械顶死,卸力剔除',
                          flush=True)
                    return None
            else:
                self.slow_hits = 0
            time.sleep(DT)
        # 终点保持,等待到位
        t_end = time.time() + SETTLE
        last = None
        while time.time() < t_end:
            fb = self.bus.transact(mid, 1, 0.0, 0.0, q_to, KP, KD)
            if fb:
                last = self.mt.log(mid, phase + '/到位', q_to, fb)
                if not self.check(mid, fb):
                    return None
            time.sleep(0.05)
        return last

    def read_q0(self, mid):
        qs = []
        for _ in range(3):
            fb = self.bus.probe(mid)
            if fb:
                if not self.check(mid, fb):
                    return None
                qs.append(fb['outputPos'])
            time.sleep(0.04)
        return statistics.median(qs) if qs else None

    def release(self, mid):
        self.bus.release(mid, kd=0.3)

    def play_round(self, mid, rnd):
        lo, hi = ANGLES[mid]
        self.stall_hits = 0
        self.slow_hits = 0
        self.stalled = False
        a_cw = random.uniform(lo, hi)    # 顺时针角度(负方向)
        a_ccw = random.uniform(lo, hi)   # 逆时针角度(正方向),独立抽取
        q0 = self.read_q0(mid)
        if q0 is None:
            return False, '读取起始位置失败/预检中止'
        q1 = q0 - math.radians(a_cw)     # 顺时针终点
        q2 = q1 + math.radians(a_ccw)    # 逆时针终点
        print(f'[第{rnd}轮 t={self.mt_now():.1f}s] 电机{mid}: 顺时针 {a_cw:.1f}° → '
              f'逆时针 {a_ccw:.1f}° → 复位到 {math.degrees(q0):.1f}°', flush=True)
        if self.move(mid, q0, q1, f'R{rnd}顺时针') is None:
            self.release(mid); return False, '顺时针动作中止'
        if self.move(mid, q1, q2, f'R{rnd}逆时针') is None:
            self.release(mid); return False, '逆时针动作中止'
        fb = self.move(mid, q2, q0, f'R{rnd}复位')
        if fb is None:
            self.release(mid); return False, '复位动作中止'
        err = math.degrees(fb['outputPos'] - q0)
        print(f'    完成: 复位偏差 {err:+.2f}°,壳温 {fb["Temp"]}°C,电压 {fb["vol"]:.1f}V', flush=True)
        return True, None

    def mt_now(self):
        return time.time() - self.mt.t0


def main():
    ap = argparse.ArgumentParser(description='电机 7/8/14 随机运转 1 分钟')
    ap.add_argument('--port', default='/dev/ttyACM0')
    ap.add_argument('--baud', type=int, default=6000000)
    ap.add_argument('--duration', type=float, default=DURATION)
    ap.add_argument('--pause', type=float, default=PAUSE)
    ap.add_argument('--seed', type=int, default=None, help='随机种子(复现实验用)')
    ap.add_argument('--speed', choices=sorted(SPEED_TIERS), default='slowest',
                    help='转速档: fast=100°/s medium=50°/s slow=25°/s slowest=12.5°/s(默认)')
    ap.add_argument('--steps-per-180', type=int, default=None, help='自定义每180°步数(覆盖 --speed)')
    ap.add_argument('--csv', default='/tmp/motor_random_log.csv')
    a = ap.parse_args()
    if a.seed is not None:
        random.seed(a.seed)
    global STEPS_PER_180
    STEPS_PER_180 = a.steps_per_180 if a.steps_per_180 else SPEED_TIERS[a.speed]

    bus = Bus(a.port, a.baud)
    mt = MotorTest(bus, a.csv)
    run = Runner(bus, mt)
    print(f'串口: {a.port} @ {a.baud} bps | 候选电机 {sorted(ANGLES)} | 总时长 {a.duration:.0f}s | 轮间隔 {a.pause:.0f}s')
    print(f'转速: {a.speed if not a.steps_per_180 else f"自定义{STEPS_PER_180}步/180°"} '
          f'({STEPS_PER_180}步/180° ≈ {180.0 / (STEPS_PER_180 * DT):.1f}°/s)')

    # 预检: 三台电机都探测一遍
    alive = []
    for mid in sorted(ANGLES):
        fb = bus.probe(mid)
        if fb is None:
            print(f'  电机{mid}: 离线,剔除')
        elif fb['MError'] != 0:
            print(f'  电机{mid}: 有故障码 MError={fb["MError"]},剔除')
        elif fb['Temp'] >= TEMP_ABORT:
            print(f'  电机{mid}: 壳温 {fb["Temp"]}°C 已达中止阈值,剔除')
        else:
            alive.append(mid)
            print(f'  电机{mid}: 在线,位置 {math.degrees(fb["outputPos"]):.1f}°,壳温 {fb["Temp"]}°C,电压 {fb["vol"]:.1f}V')
    if not alive:
        print('没有可运行的电机,退出'); bus.close(); return 2

    excluded, rnd = set(), 0
    t_start = time.time()
    try:
        while time.time() - t_start < a.duration:
            cands = [m for m in alive if m not in excluded]
            if not cands:
                print('所有电机均被剔除,提前结束'); break
            mid = random.choice(cands)
            rnd += 1
            try:
                ok, why = run.play_round(mid, rnd)
            except Exception as e:
                ok, why = False, f'异常: {e!r}'
                try:
                    run.release(mid)
                except Exception:
                    pass
            if not ok:
                excluded.add(mid)
                print(f'  电机{mid} 退出后续轮次({why});剩余候选 {sorted(set(alive) - excluded)}')
            # 轮间隔(不超出总时长的部分照睡,超时则直接收尾)
            left = a.duration - (time.time() - t_start)
            if left > 0:
                time.sleep(min(a.pause, left))
    except KeyboardInterrupt:
        print('\n用户中断')
    finally:
        # 收尾: 全部卸力
        for mid in alive:
            try:
                run.release(mid)
            except Exception:
                pass
        elapsed = time.time() - t_start
        print(f'\n运行结束: 共 {rnd} 轮 / {elapsed:.1f}s,被剔除电机: {sorted(excluded) or "无"}')
        print(f'峰值扭矩 {mt.peak_torque:.3f} N·m | 最高壳温 {mt.max_temp}°C | 最低电压 {mt.min_volt:.1f}V')
        mt.save_csv()
        bus.close()
    return 0


if __name__ == '__main__':
    sys.exit(main())
