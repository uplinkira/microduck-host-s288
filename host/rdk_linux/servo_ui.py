#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
舵机限位调试 UI(J288/S288 总线)

  python3 servo_ui.py                      # 默认 /dev/ttyACM0 @ 6Mbps
  python3 servo_ui.py --port /dev/ttyUSB0
  python3 servo_ui.py --sim                # 离线预览,不连硬件

功能:
  - 每台电机(7 仰头 / 8 摇头 / 14 嘴部)独立的顺时针/逆时针软限位滑条
  - 每台电机独立的位置滑条,范围由软限位实时夹紧
  - 全局速度滑条 2~60 °/s,默认 12.5 °/s(低速)
  - 标定中位 / 回中 / 使能 / 卸力,以及一键全部卸力(急停)
  - 实时显示位置、壳温、电压、扭矩、错误码

安全策略(与 motor_random.py 一致,阈值见 仓库 README 与本文件参数表):
  - timeout 控制位恒 0;Kp=10/Kd=0.5(输出端量纲)
  - MError!=0 或壳温>=TEMP_ABORT: 立即卸力并禁用该电机
  - 连续 STALL_N 次位置误差 >STALL_DEG: 判机械阻挡,卸力禁用
  - 连续 3 次无反馈: 判通信丢失,禁用
  - 退出/急停/异常路径一律显式卸力(timeout 位为 0,不卸力电机会一直顶住)
"""
import sys, os, time, math, threading, argparse, statistics, queue

if hasattr(sys.stdout, 'reconfigure'):          # 打包成无控制台 exe 时 stdout 是 None
    sys.stdout.reconfigure(encoding='utf-8', errors='replace')
_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)
sys.path.insert(0, os.path.join(os.path.dirname(_HERE), 'common'))   # motor_sweep 在 host/common/

import tkinter as tk
from tkinter import ttk

# ---- 底层总线(motor_sweep.py 缺失时自动退化为离线仿真) ----
BUS_IMPORT_ERR = None
try:
    from motor_sweep import Bus, TEMP_ABORT, TEMP_WARN, VOLT_WARN, TORQUE_WARN
except Exception as e:                      # noqa: BLE001
    BUS_IMPORT_ERR = e
    Bus = None
    TEMP_ABORT, TEMP_WARN, VOLT_WARN, TORQUE_WARN = 85, 70, 11.0, 3.0

# ---- 限位表: 详见 仓库 README 与本文件参数表,改这里即可 ----
# cw = 顺时针 = 负方向, ccw = 逆时针 = 正方向
# 已知关节的语义与限位。总线上出现但不在此表的 ID,按 UNKNOWN_SPEC 给保守默认值
KNOWN = {
    7:  dict(name='仰头', cw='向上后仰', ccw='向下低头',
             cw_max=12.0, ccw_max=12.0, cw_def=8.0,  ccw_def=8.0),
    8:  dict(name='摇头', cw='向右',     ccw='向左',
             cw_max=45.0, ccw_max=45.0, cw_def=30.0, ccw_def=30.0),
    # 2026-09-23 实测: 闭合方向约 13° 即顶死,原 70/60 会硬顶限位,已收紧
    14: dict(name='嘴部', cw='向上闭合', ccw='向下张开',
             cw_max=15.0, ccw_max=15.0, cw_def=8.0,  ccw_def=8.0),
}
# 未知关节: 不知道机械行程,给最保守的值,必须现场标定
UNKNOWN_SPEC = dict(name='未知关节', cw='顺时针', ccw='逆时针',
                    cw_max=15.0, ccw_max=15.0, cw_def=3.0, ccw_def=3.0)
DEFAULT_SIM_IDS = [6, 7, 8, 14]      # 仿真模式下没有总线可扫,用这组占位


def spec_for(mid, limit_max=None):
    """按 ID 取关节规格。未知 ID 返回保守默认值,并在名字里标出来。

    limit_max: 抬高限位滑条的上限(度)。默认值仍按表里的保守值,
    只是允许往上拖得更多 —— 想放开行程时用 --limit-max 指定。
    """
    s = dict(KNOWN.get(mid, UNKNOWN_SPEC))
    s['id'] = mid
    s['known'] = mid in KNOWN
    if limit_max:
        s['cw_max'] = max(s['cw_max'], float(limit_max))
        s['ccw_max'] = max(s['ccw_max'], float(limit_max))
    return s

KP, KD = 10.0, 0.5
DT = 0.02                 # 控制周期 s (50Hz,实测单机上限约 160Hz)
SPEED_DEFAULT = 12.5      # °/s
SPEED_MIN, SPEED_MAX = 2.0, 60.0
STALL_DEG = 12.0          # 堵转判定: 位置误差阈值(度)。正常运动中误差只有 0.6~4.5°,12° 有充足余量
STALL_N = 5               # 连续超阈值次数
STALL_SPD = 1.0           # 堵转判定: 输出端速度阈值(°/s),低于此视为没在动
STALL_SPD_N = 8           # 指令在动但实测不动,连续多少次判堵转(8×20ms=0.16s)
COMM_FAIL_N = 3
SMALL = ('TkDefaultFont', 9)     # 紧凑布局用的小字号
MONO = ('TkFixedFont', 10)

# --- 顺序播放 ---
# 一次只动一台电机,同一时刻只有一台通电: 峰值电流最小,对供电最友好
SEQ_IDS = (7, 8)                 # 默认参与的电机: 仰头 + 摇头
SEQ_EXCLUDE = (14,)              # 永不参与顺序播放的电机(嘴部只做手动控制)
SEQ_AMP_DEFAULT = 3.0            # 默认幅度(度),比手动标定的默认值更保守
SEQ_AMP_MAX = 8.0                # 幅度滑条上限
SEQ_DWELL_DEFAULT = 0.5          # 每个端点停留 s
SEQ_PATTERN = (-1.0, 0.0, 1.0, 0.0)   # 幅度的倍数: 顺时针 → 回中 → 逆时针 → 回中


class MotorState:
    """一台电机的共享状态。所有字段读写都在 Controller.lock 保护下。"""

    def __init__(self, spec):
        self.spec = spec
        self.mid = spec['id']
        self.center = 0.0      # 绝对零位(输出端弧度),标定中位时写入
        self.target = 0.0      # 相对中位的目标偏移(弧度)
        self.cmd = 0.0         # 相对中位的当前指令(弧度),按速度限制逼近 target
        self.enabled = False
        self.want_release = False
        self.centered = False  # 是否已标定过中位
        self.fb = None         # 最近一次反馈 dict
        self.status = '未使能'
        self.error = None      # 非 None 表示被保护逻辑禁用
        self.stall_hits = 0
        self.slow_hits = 0
        self.comm_fail = 0


class Controller:
    """后台控制线程: 按 DT 周期对所有使能电机做限速步进 + 反馈采集。"""

    def __init__(self, port, baud, sim, ids=None, limit_max=None):
        self.port, self.baud, self.sim = port, baud, sim
        self.ids = ids                      # None = 连上后自动扫描总线
        self.limit_max = limit_max          # 抬高限位滑条上限,不改默认值
        self.bus = None
        self.states = {}                    # connect() 里按实际发现的电机建立
        self.lock = threading.Lock()
        self.speed = SPEED_DEFAULT          # °/s,全局
        self.jobs = queue.Queue()           # UI → 控制线程的一次性动作
        self.running = False
        self.connected = False
        self.conn_msg = '未连接'
        self.thread = None
        self.seq_thread = None
        self.seq_stop = threading.Event()
        self.seq_msg = ''               # 播放状态,由 UI 线程轮询读取(不能跨线程改控件)
        self.seq_active = False
        self.paused = False             # 暂停: 序列内的电机停在原地保持,不卸力
        self.seq_ids = set()            # 本次序列涉及的电机,暂停只影响这些

    # ---------- 连接 ----------
    def connect(self):
        if self.sim:
            ids = self.ids or DEFAULT_SIM_IDS
            self.connected, self.conn_msg = True, '离线仿真'
        else:
            if Bus is None:
                self.conn_msg = f'motor_sweep 不可用: {BUS_IMPORT_ERR}'
                return False
            try:
                self.bus = Bus(self.port, self.baud)
            except Exception as e:          # noqa: BLE001
                self.conn_msg = f'串口打开失败: {e}'
                return False
            ids = self.ids if self.ids else self._discover()
            if not ids:
                self.conn_msg = f'{self.port}: 总线上没有电机响应'
                self.bus.close()
                return False
            self.connected, self.conn_msg = True, f'{self.port} @ {self.baud}'
        self.states = {m: MotorState(spec_for(m, self.limit_max)) for m in ids}
        self.running = True
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()
        return True

    def _discover(self):
        """扫描 ID 0~14,返回有响应且无故障码的电机列表(零力矩探测,不会让电机出力)。"""
        found = []
        for mid in range(15):
            fb = self.bus.probe(mid)
            if fb is None:
                continue
            tag = '' if mid in KNOWN else '  [未知关节]'
            print(f'  发现 ID {mid:2d}: 位置 {math.degrees(fb["outputPos"]):+8.2f}°  '
                  f'壳温 {fb["Temp"]:3d}°C  {fb["vol"]:5.1f}V  '
                  f'MError=0x{fb["MError"]:X}{tag}', flush=True)
            found.append(mid)
        return found

    def shutdown(self):
        """停线程并确保每台电机都卸力(timeout 位为 0,不卸力会一直顶住)。"""
        self.running = False
        if self.thread:
            self.thread.join(timeout=2.0)
        if self.bus:
            for mid in self.states:
                try:
                    self.bus.release(mid, kd=0.3)
                except Exception:           # noqa: BLE001
                    pass
            try:
                self.bus.close()
            except Exception:               # noqa: BLE001
                pass
        self.connected = False

    # ---------- UI 调用的接口 ----------
    def set_speed(self, deg_s):
        with self.lock:
            self.speed = max(SPEED_MIN, min(SPEED_MAX, deg_s))

    def set_target(self, mid, deg):
        with self.lock:
            self.states[mid].target = math.radians(deg)

    def set_enabled(self, mid, on):
        with self.lock:
            st = self.states[mid]
            if on:
                if st.error:
                    return False
                if not st.centered:
                    st.status = '请先标定中位'
                    return False
                st.cmd = st.target = 0.0     # 使能时从中位起步,避免突跳
                st.enabled = True
                st.status = '已使能'
            else:
                st.enabled = False
                st.want_release = True
                st.status = '已卸力'
            return True

    def release_all(self):
        with self.lock:
            for st in self.states.values():
                st.enabled = False
                st.want_release = True
                st.status = '急停卸力'

    def calibrate(self, mid):
        self.jobs.put(('calib', mid))

    def clear_error(self, mid):
        with self.lock:
            st = self.states[mid]
            st.error = None
            st.stall_hits = st.slow_hits = st.comm_fail = 0
            st.status = '已复位告警'

    # ---------- 顺序播放 ----------
    def play_sequence(self, ids, amp, dwell, loop):
        """按 ids 顺序逐台做一遍 SEQ_PATTERN。同一时刻只有一台通电。"""
        if self.seq_thread and self.seq_thread.is_alive():
            return False
        self.seq_stop.clear()
        self.paused = False
        self.seq_ids = set(ids)
        self.seq_active = True
        self.seq_thread = threading.Thread(
            target=self._seq_run, args=(list(ids), amp, dwell, loop), daemon=True)
        self.seq_thread.start()
        return True

    def stop_sequence(self):
        self.seq_stop.set()
        self.paused = False             # 停止时解除暂停,否则收尾会卡住

    def toggle_pause(self):
        """暂停/继续。暂停时电机停在原地保持力矩,不卸力,不丢失进度。"""
        if not self.seq_active:
            return False
        self.paused = not self.paused
        self._say('已暂停(电机保持在原位)' if self.paused else '继续播放')
        return self.paused

    def _say(self, msg):
        with self.lock:
            self.seq_msg = msg

    def _seq_sleep(self, sec):
        """可中断的等待。暂停期间不计时。返回 False 表示被要求停止。"""
        end = time.time() + sec
        while time.time() < end:
            if self.seq_stop.is_set():
                return False
            if self.paused:
                end += 0.02
            time.sleep(0.02)
        return not self.seq_stop.is_set()

    def _seq_wait_arrive(self, mid, tgt_deg):
        """等电机走到位。任何保护触发或超时都返回 False。"""
        with self.lock:
            st = self.states[mid]
            dist = abs(math.degrees(st.cmd) - tgt_deg)
            spd = self.speed
        deadline = time.time() + dist / max(spd, 0.1) * 3.0 + 3.0
        while time.time() < deadline:
            if self.seq_stop.is_set():
                return False
            if self.paused:             # 暂停期间不计入超时
                deadline += 0.02
                time.sleep(0.02)
                continue
            with self.lock:
                st = self.states[mid]
                if st.error or not st.enabled:
                    return False
                if abs(math.degrees(st.cmd) - tgt_deg) < 0.05:
                    return True
            time.sleep(0.02)
        self._say(f'ID{mid} 走到 {tgt_deg:+.1f}° 超时')
        return False

    def _seq_one(self, mid, amp, dwell):
        """跑完一台。返回 False 表示出错需要中止整个序列。"""
        with self.lock:
            st = self.states[mid]
            if st.error:
                self.seq_msg = f'ID{mid} 有未清除的告警,跳过'
                return True
            if not st.centered:
                self.seq_msg = f'ID{mid} 未标定中位,跳过'
                return True
            spec = st.spec
        # 幅度再被该电机自己的软限位夹一次,取最小值
        a = min(amp, spec['cw_def'], spec['ccw_def'])
        if not self.set_enabled(mid, True):
            self._say(f'ID{mid} 使能失败,跳过')
            return True
        try:
            for k in SEQ_PATTERN:
                if self.seq_stop.is_set():
                    return True
                tgt = a * k
                self._say(f'ID{mid} → {tgt:+.1f}°')
                self.set_target(mid, tgt)
                if not self._seq_wait_arrive(mid, tgt):
                    with self.lock:
                        err = self.states[mid].error
                    if err:
                        self._say(f'ID{mid} {err}')
                        return False
                    return True
                if not self._seq_sleep(dwell):
                    return True
        finally:
            self.set_enabled(mid, False)     # 走完立刻卸力,下一台再通电
        return True

    def _seq_run(self, ids, amp, dwell, loop):
        rnd = 0
        try:
            while not self.seq_stop.is_set():
                rnd += 1
                for mid in ids:
                    if self.seq_stop.is_set():
                        break
                    if mid not in self.states:
                        self._say(f'ID{mid} 不在本次加载的电机里,跳过')
                        continue
                    if not self._seq_one(mid, amp, dwell):
                        with self.lock:
                            why = self.states[mid].error or '未知原因'
                        self._say(f'第{rnd}轮中止: ID{mid} {why}')
                        print(f'✗ 第{rnd}轮 ID{mid} 中止: {why}', flush=True)
                        return
                if not loop:
                    break
            self._say('已停止' if self.seq_stop.is_set() else f'播放完成({rnd}轮)')
        finally:
            for mid in ids:                  # 收尾: 参与过的电机一律卸力
                if mid in self.states:
                    self.set_enabled(mid, False)
            self.seq_active = False

    def snapshot(self):
        """给 UI 线程的只读快照。"""
        with self.lock:
            return {
                mid: dict(cmd=math.degrees(st.cmd), target=math.degrees(st.target),
                          enabled=st.enabled, centered=st.centered,
                          status=st.status, error=st.error,
                          fb=dict(st.fb) if st.fb else None)
                for mid, st in self.states.items()
            }

    # ---------- 控制线程 ----------
    def _transact(self, mid, q, v=0.0):
        """v = 前馈轨迹速度(rad/s)。不发的话 Kd×(0−实际速度) 会全程反向制动,加大跟随滞后。"""
        if self.sim:
            time.sleep(0.001)
            st = self.states[mid]
            return dict(outputPos=st.center + st.cmd, Temp=42, vol=24.0,
                        outputTor=0.05, outputSpd=v, MError=0)
        return self.bus.transact(mid, 1, 0.0, v, q, KP, KD)

    def _probe(self, mid):
        if self.sim:
            return dict(outputPos=0.0, Temp=42, vol=24.0, outputTor=0.0, MError=0)
        return self.bus.probe(mid)

    def _release(self, mid):
        if not self.sim and self.bus:
            try:
                self.bus.release(mid, kd=0.3)
            except Exception:               # noqa: BLE001
                pass

    def _fault(self, st, msg):
        """触发保护: 卸力 + 禁用 + 记录原因。调用时已持有 lock。"""
        st.enabled = False
        st.error = msg
        st.status = msg
        fb = st.fb or {}
        print(f'✗ ID{st.mid} 保护触发: {msg} | 壳温 {fb.get("Temp", "?")}°C '
              f'绕组 {fb.get("sensor", "?")}°C 扭矩 {fb.get("outputTor", 0):+.2f}N·m '
              f'电压 {fb.get("vol", 0):.1f}V', flush=True)
        self._release(st.mid)

    def _check(self, st, fb, q_cmd_abs, v=0.0):
        """返回 False 表示已触发保护。调用时已持有 lock。v = 本周期指令速度(rad/s)。"""
        if fb['MError'] != 0:
            self._fault(st, f'MError={fb["MError"]},已卸力')
            return False
        if fb['Temp'] >= TEMP_ABORT:
            self._fault(st, f'壳温 {fb["Temp"]}°C ≥ {TEMP_ABORT}°C,已卸力')
            return False
        if abs(math.degrees(fb['outputPos'] - q_cmd_abs)) > STALL_DEG:
            st.stall_hits += 1
            if st.stall_hits >= STALL_N:
                self._fault(st, f'连续{STALL_N}次误差>{STALL_DEG}°,判机械阻挡,已卸力')
                return False
        else:
            st.stall_hits = 0
        # 指令在动但实测速度≈0 → 机械顶死,比位置误差阈值早得多就能抓住
        if abs(math.degrees(v)) > 2.0 and abs(math.degrees(fb.get('outputSpd', 0.0))) < STALL_SPD:
            st.slow_hits += 1
            if st.slow_hits >= STALL_SPD_N:
                self._fault(st, f'指令在动但实测静止,扭矩 {fb["outputTor"]:+.2f}N·m,'
                                f'判机械顶死,已卸力')
                return False
        else:
            st.slow_hits = 0
        return True

    def _do_calib(self, mid):
        """读 3 次取中值作为中位,与 motor_random.read_q0 一致。"""
        qs = []
        for _ in range(3):
            fb = self._probe(mid)
            if fb:
                if fb['MError'] != 0:
                    with self.lock:
                        self._fault(self.states[mid], f'标定时 MError={fb["MError"]}')
                    return
                qs.append(fb['outputPos'])
            time.sleep(0.04)
        with self.lock:
            st = self.states[mid]
            if not qs:
                st.status = '标定失败: 无反馈'
                return
            st.center = statistics.median(qs)
            st.cmd = st.target = 0.0
            st.centered = True
            st.status = f'中位已标定 {math.degrees(st.center):.1f}°'

    def _loop(self):
        while self.running:
            t0 = time.time()

            while not self.jobs.empty():
                try:
                    kind, mid = self.jobs.get_nowait()
                except queue.Empty:
                    break
                if kind == 'calib':
                    self._do_calib(mid)

            for st in self.states.values():
                with self.lock:
                    if st.want_release:
                        st.want_release = False
                        self._release(st.mid)
                    if not st.enabled:
                        continue
                    # 暂停: 序列涉及的电机停在原地保持力矩(不卸力,不丢进度)
                    if self.paused and st.mid in self.seq_ids:
                        applied = 0.0
                    else:
                        # 限速步进: 每周期最多走 speed*DT
                        step = math.radians(self.speed) * DT
                        delta = st.target - st.cmd
                        applied = max(-step, min(step, delta))
                    st.cmd += applied
                    q = st.center + st.cmd
                    v = applied / DT          # 本周期的实际轨迹速度,前馈出去

                fb = self._transact(st.mid, q, v)

                with self.lock:
                    if fb is None:
                        st.comm_fail += 1
                        if st.comm_fail >= COMM_FAIL_N:
                            self._fault(st, f'连续{COMM_FAIL_N}次无反馈,通信丢失')
                        continue
                    st.comm_fail = 0
                    st.fb = fb
                    self._check(st, fb, q, v)

            time.sleep(max(0.0, DT - (time.time() - t0)))


class MotorPanel(ttk.LabelFrame):
    """单台电机的一组控件。"""

    def __init__(self, master, ctrl, spec):
        # 方向说明并进标题,省掉一整行
        super().__init__(master, padding=4, text=f'ID {spec["id"]}  {spec["name"]}   '
                                                f'顺(−){spec["cw"]} / 逆(+){spec["ccw"]}')
        self.ctrl, self.spec = ctrl, spec
        self.mid = spec['id']
        self._syncing = False
        sk = dict(orient='horizontal', resolution=0.5, width=9, sliderlength=16,
                  font=SMALL, borderwidth=1)

        # --- 第一行: 两个软限位并排 ---
        ttk.Label(self, text='顺限位', font=SMALL).grid(row=0, column=0, sticky='w')
        self.cw = tk.DoubleVar(value=spec['cw_def'])
        self.s_cw = tk.Scale(self, from_=0, to=spec['cw_max'], variable=self.cw,
                             length=150, **sk)
        self.s_cw.grid(row=0, column=1, sticky='we')

        ttk.Label(self, text='逆限位', font=SMALL).grid(row=0, column=2, sticky='w', padx=(8, 0))
        self.ccw = tk.DoubleVar(value=spec['ccw_def'])
        self.s_ccw = tk.Scale(self, from_=0, to=spec['ccw_max'], variable=self.ccw,
                              length=150, **sk)
        self.s_ccw.grid(row=0, column=3, sticky='we')

        # --- 第二行: 位置 ---
        ttk.Label(self, text='位置', font=SMALL).grid(row=1, column=0, sticky='w')
        self.pos = tk.DoubleVar(value=0.0)
        self.s_pos = tk.Scale(self, from_=-spec['cw_def'], to=spec['ccw_def'],
                              variable=self.pos, length=340, **sk)
        self.s_pos.grid(row=1, column=1, columnspan=3, sticky='we')

        # --- 第三行: 按钮与读数同行 ---
        bar = ttk.Frame(self)
        bar.grid(row=2, column=0, columnspan=4, sticky='we', pady=(2, 0))
        self.b_cal = ttk.Button(bar, text='标定', width=5, command=self._calib)
        self.b_cal.pack(side='left')
        self.b_en = ttk.Button(bar, text='使能', width=5, command=self._enable)
        self.b_en.pack(side='left', padx=2)
        self.b_rel = ttk.Button(bar, text='卸力', width=5, command=self._disable)
        self.b_rel.pack(side='left')
        ttk.Button(bar, text='回中', width=5, command=self._home).pack(side='left', padx=2)
        ttk.Button(bar, text='清警', width=5, command=self._clear).pack(side='left')
        self.lbl = ttk.Label(bar, text='—', font=MONO)
        self.lbl.pack(side='left', padx=(10, 0))

        self.columnconfigure(1, weight=1)
        self.columnconfigure(3, weight=1)

        # 全部控件就位后再挂回调,避免构造期间 Scale 触发 command 时引用到尚未创建的滑条
        self.s_cw.config(command=lambda _v: self._limits_changed())
        self.s_ccw.config(command=lambda _v: self._limits_changed())
        self.s_pos.config(command=self._pos_changed)

    # --- 事件 ---
    def _limits_changed(self):
        """软限位变化时重设位置滑条量程,并把越界的目标夹回来。"""
        lo, hi = -self.cw.get(), self.ccw.get()
        self.s_pos.config(from_=lo, to=hi)
        v = max(lo, min(hi, self.pos.get()))
        if v != self.pos.get():
            self.pos.set(v)
        self.ctrl.set_target(self.mid, v)

    def _pos_changed(self, _v=None):
        if self._syncing:
            return
        v = max(-self.cw.get(), min(self.ccw.get(), self.pos.get()))
        self.ctrl.set_target(self.mid, v)

    def _calib(self):
        self.ctrl.calibrate(self.mid)

    def _enable(self):
        self._home()
        self.ctrl.set_enabled(self.mid, True)

    def _disable(self):
        self.ctrl.set_enabled(self.mid, False)

    def _home(self):
        self._syncing = True
        self.pos.set(0.0)
        self._syncing = False
        self.ctrl.set_target(self.mid, 0.0)

    def _clear(self):
        self.ctrl.clear_error(self.mid)

    def refresh(self, snap, playing=False):
        # 播放中目标由序列驱动,把滑条同步过去,否则看不出电机在动
        if playing and abs(self.pos.get() - snap['target']) > 0.05:
            self._syncing = True
            lo, hi = -self.cw.get(), self.ccw.get()
            self.pos.set(max(lo, min(hi, snap['target'])))
            self._syncing = False
        fb, parts = snap['fb'], []
        parts.append(f'{snap["cmd"]:+5.1f}°')
        if fb:
            parts.append(f'实{math.degrees(fb["outputPos"]):+7.1f}°')
            parts.append(f'{fb["Temp"]:3d}°C')
            parts.append(f'{fb["vol"]:4.1f}V')
            parts.append(f'{fb["outputTor"]:+5.2f}N·m')
        else:
            parts.append('无反馈')
        parts.append(snap['error'] or snap['status'])
        self.lbl.config(text=' | '.join(parts),
                        foreground='#b00' if snap['error'] else '#000')
        self.b_en.state(['disabled'] if snap['enabled'] or snap['error'] else ['!disabled'])
        self.b_rel.state(['!disabled'] if snap['enabled'] else ['disabled'])
        if fb:
            if fb['Temp'] >= TEMP_WARN or fb['vol'] < VOLT_WARN or abs(fb['outputTor']) > TORQUE_WARN:
                self.lbl.config(foreground='#c60' if not snap['error'] else '#b00')


class App(tk.Tk):
    def __init__(self, ctrl):
        super().__init__()
        self.ctrl = ctrl
        self.title('xi-an-duck 上位机调试')
        self.protocol('WM_DELETE_WINDOW', self._quit)

        top = ttk.Frame(self, padding=10)
        top.pack(fill='x')
        mode = '离线仿真(不连硬件)' if ctrl.sim else ctrl.conn_msg
        ttk.Label(top, text=f'连接: {mode}',
                  foreground='#a60' if ctrl.sim else '#060').pack(side='left')
        ttk.Button(top, text='■ 全部卸力(急停)', command=ctrl.release_all).pack(side='right')

        spd = ttk.LabelFrame(self, text='全局速度', padding=8)
        spd.pack(fill='x', padx=10)
        self.speed = tk.DoubleVar(value=SPEED_DEFAULT)
        tk.Scale(spd, from_=SPEED_MIN, to=SPEED_MAX, resolution=0.5, orient='horizontal',
                 variable=self.speed, length=560,
                 command=lambda _v: ctrl.set_speed(self.speed.get())).pack(fill='x')
        ttk.Label(spd, foreground='#666',
                  text=f'默认 {SPEED_DEFAULT} °/s。未使用双向电源/放电电阻前不要超过 50 °/s'
                       f'(高速制动会反灌母线),详见 仓库 README 与本文件参数表').pack(anchor='w')

        # --- 顺序播放 ---
        seq = ttk.LabelFrame(self, text='顺序播放(一次只动一台,同一时刻只有一台通电)', padding=6)
        seq.pack(fill='x', padx=10, pady=(6, 0))
        r1 = ttk.Frame(seq); r1.pack(fill='x')
        ttk.Label(r1, text='参与:', font=SMALL).pack(side='left')
        self.seq_vars = {}
        for mid in sorted(ctrl.states):
            if mid in SEQ_EXCLUDE:          # 嘴部等只做手动控制,不进序列
                continue
            v = tk.BooleanVar(value=mid in SEQ_IDS)
            self.seq_vars[mid] = v
            ttk.Checkbutton(r1, text=f'{mid}', variable=v).pack(side='left', padx=(2, 4))
        excl = [m for m in sorted(ctrl.states) if m in SEQ_EXCLUDE]
        if excl:
            ttk.Label(r1, text=f'(ID {",".join(map(str, excl))} 仅手动)',
                      font=SMALL, foreground='#a60').pack(side='left', padx=(2, 4))
        ttk.Label(r1, text='幅度', font=SMALL).pack(side='left', padx=(10, 0))
        self.seq_amp = tk.DoubleVar(value=SEQ_AMP_DEFAULT)
        tk.Scale(r1, from_=1.0, to=SEQ_AMP_MAX, resolution=0.5, orient='horizontal',
                 variable=self.seq_amp, length=110, width=9, sliderlength=16,
                 font=SMALL).pack(side='left')
        # 滑条之外再给一个输入框,可以敲精确值,也可以超过滑条上限
        ttk.Spinbox(r1, from_=0.5, to=180.0, increment=0.5, width=5, font=SMALL,
                    textvariable=self.seq_amp).pack(side='left', padx=(2, 0))
        ttk.Label(r1, text='停留', font=SMALL).pack(side='left', padx=(8, 0))
        self.seq_dwell = tk.DoubleVar(value=SEQ_DWELL_DEFAULT)
        tk.Scale(r1, from_=0.0, to=3.0, resolution=0.1, orient='horizontal',
                 variable=self.seq_dwell, length=90, width=9, sliderlength=16,
                 font=SMALL).pack(side='left')
        self.seq_loop = tk.BooleanVar(value=False)
        ttk.Checkbutton(r1, text='循环', variable=self.seq_loop).pack(side='left', padx=(8, 0))
        self.b_play = ttk.Button(r1, text='播放', width=6, command=self._play)
        self.b_play.pack(side='left', padx=(10, 2))
        self.b_pause = ttk.Button(r1, text='暂停', width=6, command=self._pause)
        self.b_pause.pack(side='left', padx=(0, 2))
        self.b_stop = ttk.Button(r1, text='停止', width=6, command=ctrl.stop_sequence)
        self.b_stop.pack(side='left')
        self.seq_lbl = ttk.Label(seq, text='就绪。播放前每台都要先标定中位', font=SMALL,
                                 foreground='#666')
        self.seq_lbl.pack(anchor='w', pady=(3, 0))

        # 电机面板放进可滚动区域,电机多时不会把窗口撑到屏幕外
        wrap = ttk.Frame(self)
        wrap.pack(fill='both', expand=True, padx=10, pady=(6, 0))
        canvas = tk.Canvas(wrap, highlightthickness=0, height=460)
        sb = ttk.Scrollbar(wrap, orient='vertical', command=canvas.yview)
        inner = ttk.Frame(canvas)
        win = canvas.create_window((0, 0), window=inner, anchor='nw')
        canvas.configure(yscrollcommand=sb.set)
        canvas.pack(side='left', fill='both', expand=True)
        sb.pack(side='right', fill='y')
        inner.bind('<Configure>', lambda e: canvas.configure(scrollregion=canvas.bbox('all')))
        canvas.bind('<Configure>', lambda e: canvas.itemconfig(win, width=e.width))
        for seq in ('<MouseWheel>', '<Button-4>', '<Button-5>'):
            canvas.bind_all(seq, lambda e: canvas.yview_scroll(
                -1 if getattr(e, 'num', 0) == 4 or getattr(e, 'delta', 0) > 0 else 1, 'units'))

        self.panels = {}
        for mid in sorted(ctrl.states):
            p = MotorPanel(inner, ctrl, ctrl.states[mid].spec)
            p.pack(fill='x', pady=3)
            self.panels[mid] = p

        ttk.Label(self, padding=(10, 0, 10, 10), foreground='#666',
                  text='流程: 标定中位 → 速度压到最低 → 限位调小 → 使能 → 位置滑条小步试探。'
                       '限位值为保守默认,须按 仓库 README 的控制边界及单关节实测结果实测标定。').pack(anchor='w')

        self.after(100, self._tick)

    def _play(self):
        ids = [m for m in sorted(self.seq_vars) if self.seq_vars[m].get()]
        if not ids:
            self.seq_lbl.config(text='没有勾选任何电机', foreground='#b00')
            return
        bad = [m for m in ids if not self.ctrl.snapshot()[m]['centered']]
        if bad:
            self.seq_lbl.config(text=f'ID {bad} 还没标定中位,先点各自的“标定”', foreground='#b00')
            return
        self.ctrl.play_sequence(ids, self.seq_amp.get(), self.seq_dwell.get(),
                                self.seq_loop.get())

    def _pause(self):
        self.ctrl.toggle_pause()

    def _tick(self):
        snap = self.ctrl.snapshot()
        playing = self.ctrl.seq_active
        paused = self.ctrl.paused
        for mid, p in self.panels.items():
            # 暂停时不同步滑条,免得把用户正在拖的位置覆盖掉
            p.refresh(snap[mid], playing and not paused)
        msg = self.ctrl.seq_msg
        if msg:
            self.seq_lbl.config(text=msg,
                                foreground='#a60' if paused else ('#06c' if playing else '#666'))
        self.b_play.state(['disabled'] if playing else ['!disabled'])
        self.b_pause.state(['!disabled'] if playing else ['disabled'])
        self.b_pause.config(text='继续' if paused else '暂停')
        self.b_stop.state(['!disabled'] if playing else ['disabled'])
        self.after(100, self._tick)

    def _quit(self):
        self.ctrl.release_all()
        time.sleep(0.1)
        self.ctrl.shutdown()
        self.destroy()


def default_port():
    """跨平台猜串口: macOS 用 /dev/cu.*(tty.* 会阻塞等 DCD),Linux 用 /dev/ttyACM0。"""
    import glob
    if sys.platform == 'darwin':
        for pat in ('/dev/cu.usbmodem*', '/dev/cu.usbserial*', '/dev/cu.wchusbserial*'):
            hits = sorted(glob.glob(pat))
            if hits:
                return hits[0]
        return '/dev/cu.usbmodem*(未插入设备)'
    return '/dev/ttyACM0'


def main():
    global SEQ_AMP_MAX
    ap = argparse.ArgumentParser(description='舵机限位调试 UI')
    ap.add_argument('--port', default=default_port())
    ap.add_argument('--baud', type=int, default=6000000)
    ap.add_argument('--sim', action='store_true', help='离线仿真,不连硬件')
    ap.add_argument('--ids', help='只控制这些 ID(逗号分隔),默认自动扫描总线')
    ap.add_argument('--limit-max', type=float, default=None,
                    help='抬高每台电机限位滑条的上限(度)。默认值不变,只是能往上拖更多')
    ap.add_argument('--amp-max', type=float, default=SEQ_AMP_MAX,
                    help=f'顺序播放幅度滑条的上限(度),默认 {SEQ_AMP_MAX}')
    a = ap.parse_args()
    ids = [int(x) for x in a.ids.split(',')] if a.ids else None
    SEQ_AMP_MAX = a.amp_max

    sim = a.sim
    if not sim and Bus is None:
        print(f'⚠ 无法导入 motor_sweep ({BUS_IMPORT_ERR}),自动切换到离线仿真模式。')
        print('  接实机前请把 motor_sweep.py 放到本目录。')
        sim = True

    ctrl = Controller(a.port, a.baud, sim, ids, a.limit_max)
    print('扫描总线...' if ids is None and not sim else f'指定电机: {ids or DEFAULT_SIM_IDS}')
    if not ctrl.connect():
        print(f'✗ {ctrl.conn_msg}')
        return 2
    unknown = [m for m in sorted(ctrl.states) if not ctrl.states[m].spec['known']]
    print(f'✓ {ctrl.conn_msg} | 控制 {sorted(ctrl.states)} | 速度默认 {SPEED_DEFAULT} °/s '
          f'| 控制周期 {DT * 1000:.0f} ms')
    if unknown:
        print(f'⚠ ID {unknown} 不在已知关节表中,限位已设为最保守值(±3°),必须现场标定')

    app = App(ctrl)
    try:
        app.mainloop()
    finally:
        ctrl.release_all()
        ctrl.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
