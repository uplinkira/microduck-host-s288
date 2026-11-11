# -*- coding: utf-8 -*-
"""
标零与位姿记录的控制核心，不含界面。pose_ui.py（PyQt）在它上面画窗口。

  PoseController  在 rdk_linux/servo_ui.Controller 之上加零位持久化、整机走位姿、minjerk 平滑过渡
  PoseLibrary     命名位姿列表：记录、插中间位姿、顺序播放、存取 JSON
"""
import sys, os, time, math, json, threading, csv, tempfile
from datetime import datetime

HERE = os.path.dirname(os.path.abspath(__file__))
HOST = os.path.dirname(HERE)
for p in (os.path.join(HOST, 'common'), os.path.join(HOST, 'rdk_linux')):
    if p not in sys.path:
        sys.path.insert(0, p)

import servo_ui as su                                      # noqa: E402
from servo_ui import Controller, MotorState                # noqa: E402
from motor_sweep import POS_MAX, RATIO, KP_MAX, KD_MAX, SPD_MAX  # 协议编码边界，不是机械限位

# 打包成 exe 后，零位和位姿文件放在 exe 旁边，不放进临时解包目录
DATA_DIR = os.path.dirname(sys.executable) if getattr(sys, 'frozen', False) else HERE
CALIB_FILE = os.path.join(DATA_DIR, 'calib_host_xianduck.json')
POSE_FILE = os.path.join(DATA_DIR, 'poses_xianduck.json')

# 关节表：方案 D（装机时的编号）。腿部语义按手绘装配图推测，待实测确认后改 verified=True
# 最新模型的头部为 pitch/yaw/mouth；没有已安装 head_roll。上位机不设置角度软限位。
JOINTS = {
    1:  dict(name='右髋偏航', joint='right_hip_yaw', group='右腿', verified=False),
    2:  dict(name='右髋横滚', joint='right_hip_roll', group='右腿', verified=False),
    3:  dict(name='右髋俯仰', joint='right_hip_pitch', group='右腿', verified=False),
    4:  dict(name='右膝', joint='right_knee', group='右腿', verified=False),
    5:  dict(name='右踝', joint='right_ankle', group='右腿', verified=False),
    9:  dict(name='左髋偏航', joint='left_hip_yaw', group='左腿', verified=False),
    10: dict(name='左髋横滚', joint='left_hip_roll', group='左腿', verified=False),
    11: dict(name='左髋俯仰', joint='left_hip_pitch', group='左腿', verified=False),
    12: dict(name='左膝', joint='left_knee', group='左腿', verified=False),
    13: dict(name='左踝', joint='left_ankle', group='左腿', verified=False),
    6:  dict(name='颈俯仰', joint='neck_pitch', group='头颈', verified=False),
    7:  dict(name='头俯仰', joint='head_pitch', group='头颈', verified=True),
    8:  dict(name='头偏航', joint='head_yaw', group='头颈', verified=True),
    14: dict(name='嘴', joint='mouth', group='头颈', verified=True),
    0:  dict(name='备用', joint=None, group='其他', verified=False),
}
DEFAULT_IDS = [1, 2, 3, 4, 5, 9, 10, 11, 12, 13, 6, 7, 8, 14]
GROUP_ORDER = ['右腿', '左腿', '头颈', '其他']
VOLT_HIGH = 12.4           # S288 上限 12.6 V，3S 满电就顶着上限，高于此橙色提醒
STILL_DEG = 0.5            # 标零前 3 次读数极差超过这个值视为没静止（输出编码器只在静止时可靠）
# 刚度档位（输出端 N·m/rad，N·m·s/rad）。软：手摆微调；中：原默认；硬：保持展演姿势
GAINS = {'软': (3.0, 0.3), '中': (10.0, 0.5), '硬': (20.0, 1.0)}
TWO_PI = 2 * math.pi
TEMP_WARN, VOLT_WARN, TORQUE_WARN = su.TEMP_WARN, su.VOLT_WARN, su.TORQUE_WARN
SPEED_DEFAULT, SPEED_MIN, SPEED_MAX = su.SPEED_DEFAULT, 0.0, math.degrees(SPD_MAX)
FLASH_ZERO_SUPPORTED = False
FLASH_ZERO_REASON = '已核对指定厂家资料包；模式 2 是编码器校准注释，未提供机械原点 Flash 写入及读回接口'


def atomic_json(path, data):
    """同目录临时文件写完再替换；写失败保留原文件，所有文本为 UTF-8。"""
    path = os.path.abspath(path)
    fd, tmp = tempfile.mkstemp(prefix='.xianduck_', suffix='.tmp', dir=os.path.dirname(path))
    try:
        with os.fdopen(fd, 'w', encoding='utf-8') as f:
            json.dump(data, f, ensure_ascii=False, indent=2, allow_nan=False)
            f.flush()
            os.fsync(f.fileno())
        os.replace(tmp, path)
    finally:
        if os.path.exists(tmp):
            os.unlink(tmp)


def port_details():
    from serial.tools import list_ports as lp
    return [(p.device, p.description) for p in sorted(lp.comports())]


def wrap_pi(a):
    """把角度折到 (-π, π]。"""
    return (a + math.pi) % TWO_PI - math.pi


def spec_for(mid):
    j = JOINTS.get(mid, dict(name=f'ID{mid}', joint=None, group='其他', verified=False))
    return dict(id=mid, **j, known=mid in JOINTS)


def list_ports():
    try:
        from serial.tools import list_ports as lp
        return [p.device for p in lp.comports()]
    except Exception:                       # noqa: BLE001
        return []


def default_port():
    ps = list_ports()
    if sys.platform == 'win32':
        return ps[0] if ps else 'COM3'
    return su.default_port()


def minjerk(s):
    """最小加加速度曲线，0→1，首末速度和加速度都为 0。"""
    s = max(0.0, min(1.0, s))
    return s * s * s * (10 - 15 * s + 6 * s * s)


def interp(a, b, s):
    """两组角度按 minjerk 进度 s 插值，缺的关节按另一侧的值补。"""
    k = minjerk(s)
    out = {}
    for m in set(a) | set(b):
        x, y = a.get(m, b.get(m, 0.0)), b.get(m, a.get(m, 0.0))
        out[m] = round(x + (y - x) * k, 2)
    return out


class PoseController(Controller):
    """零位持久化、按实际读数取角、整机走位姿、平滑过渡。"""

    def __init__(self, port, baud, sim, ids, watchdog=True):
        super().__init__(port, baud, sim, ids, limit_max=None)
        self.calib = {}                     # mid -> dict(ex_zero, when)
        self.signs = {}                     # mid -> +1 / -1，策略坐标系相对上位机坐标系的符号
        self.ranges = {}                    # mid -> [min_deg, max_deg] 相对零位，录行程得到
        self.watchdog = watchdog            # True: 控制帧超时位置 1，1 s 收不到指令舵机自动停
        self.gain = '中'
        self.kp, self.kd = GAINS[self.gain]
        self.timeout_cleared = 0            # 清过几次超时锁存，给界面看
        self.range_rec = False
        self._range_thread = None
        self.states = {m: MotorState(spec_for(m)) for m in (ids or DEFAULT_IDS)}
        self.io_lock = threading.RLock()
        self.last_error = ''
        self.motion_blocked = False
        self.measurement_active = False
        self.measurement_stop = threading.Event()
        self.measurement_lock = threading.RLock()
        self.measurement_runner = None

    def set_gain(self, name):
        self.set_gain_values(*GAINS[name])
        self.gain = name

    def set_gain_values(self, kp, kd):
        if not all(math.isfinite(x) for x in (kp, kd)) or not 0 <= kp <= KP_MAX or not 0 <= kd <= KD_MAX:
            raise ValueError('Kp/Kd 超出 S288 协议范围')
        with self.lock:
            self.kp, self.kd = kp, kd
            self.gain = '自定义'

    def set_speed(self, deg_s):
        if not math.isfinite(deg_s) or not 0 <= deg_s <= SPEED_MAX:
            raise ValueError('速度超出 S288 协议范围')
        with self.lock:
            self.speed = deg_s

    def _transact(self, mid, q, v=0.0):
        """带看门狗的控制帧：超时位 = watchdog，进程崩溃后舵机 1 s 内自停。
        舵机一旦触发超时会锁存并忽略后续超时位 = 1 的帧，必须补发一帧超时位 0 清除。"""
        if self.sim:
            fb = super()._transact(mid, q, v)
            fb['vol'] = 11.8                       # 仿真按 3S 电池的正常电压，别触发高压提醒
            return fb
        fb = self.bus.transact(mid, 1, 0.0, v, q, self.kp, self.kd, timeout_bit=1 if self.watchdog else 0)
        if fb and fb.get('timeout'):
            fb = self.bus.transact(mid, 1, 0.0, v, q, self.kp, self.kd, timeout_bit=0) or fb
            self.timeout_cleared += 1
            with self.lock:
                st = self.states.get(mid)
                if st:
                    st.status = '超时锁存已清除'
        return fb

    def _probe(self, mid):
        if not self.connected or self.measurement_active:
            return None
        with self.io_lock:
            if not self.connected or self.measurement_active:
                return None
            fb = super()._probe(mid)
        if self.sim and fb:
            fb['vol'] = 11.8
        return fb

    def validate_target(self, mid, deg):
        """只校验有限数和有符号 32 位位置编码，绝不按关节/采集行程裁剪。"""
        deg = float(deg)
        if mid not in self.states or not math.isfinite(deg):
            raise ValueError('电机 ID 无效或角度不是有限数')
        absolute = self.states[mid].center + math.radians(deg)
        if not math.isfinite(absolute) or abs(absolute) > POS_MAX:
            raise ValueError(f'ID {mid} 绝对目标超出 S288 协议位置编码范围')
        raw = round(absolute * RATIO * 32768 / (2 * math.pi))
        if not -2147483648 <= raw <= 2147483647:
            raise ValueError(f'ID {mid} 绝对目标超出 32 位位置编码范围')
        return deg

    def set_target(self, mid, deg):
        if not self.connected or self.measurement_active:
            return False
        try:
            deg = self.validate_target(mid, deg)
        except ValueError as e:
            if mid in self.states:
                self.states[mid].status = str(e)
            return False
        super().set_target(mid, deg)
        return True

    def set_enabled(self, mid, on):
        """使能保持当前实测角度，不把零角当作起始指令。"""
        with self.lock:
            st = self.states[mid]
            if on:
                if self.measurement_active:
                    st.status = '实测采集中，普通运动已锁定'
                    return False
                if self.range_rec or not self.connected or not st.fb or not st.centered or st.error:
                    st.status = '需在线且已标定，无故障'
                    return False
                st.cmd = st.target = st.fb['outputPos'] - st.center
                self.motion_blocked = False
                st.enabled, st.status = True, '已使能'
            else:
                st.enabled, st.want_release, st.status = False, True, '已卸力'
            return True

    def release_all(self):
        # 急停锁存阻止尚未退出的播放线程再次使能。
        self.measurement_stop.set()
        with self.lock:
            self.motion_blocked = True
            for st in self.states.values():
                st.enabled, st.want_release, st.status = False, True, '已卸力'

    # ---------- 录行程 ----------
    def start_range(self, mids=None):
        """卸力状态下手摆，20 ms 采样每台相对零位的 min / max。"""
        if self.range_rec or not self.connected or self.measurement_active:
            return False
        mids = [m for m in (mids or self.states)
                if not self.states[m].enabled and self.states[m].centered and self.states[m].fb]
        if not mids:
            return False
        for m in mids:
            self.ranges[m] = [1e9, -1e9]
        self.range_rec = True
        def run():
            while self.range_rec:
                for m in mids:
                    fb = self._probe(m)
                    if not fb:
                        continue
                    d = math.degrees(fb['outputPos'] - self.states[m].center)
                    r = self.ranges[m]
                    r[0], r[1] = min(r[0], d), max(r[1], d)
                    with self.lock:
                        self.states[m].fb = fb
                        self.states[m].status = f'录行程 {r[0]:+.0f}..{r[1]:+.0f}°'
                time.sleep(su.DT)
        self._range_thread = threading.Thread(target=run, daemon=True)
        self._range_thread.start()
        return True

    def stop_range(self):
        self.range_rec = False
        if self._range_thread:
            self._range_thread.join()
        bad = [m for m, r in self.ranges.items() if r[0] >= r[1]]
        for m in bad:
            self.ranges.pop(m, None)
            with self.lock:
                if m in self.states:
                    self.states[m].status = '行程太小，未记录'
        for m, r in self.ranges.items():
            r[0], r[1] = round(r[0], 1), round(r[1], 1)
            with self.lock:
                if m in self.states:
                    self.states[m].status = f'行程 {r[0]:+.0f}..{r[1]:+.0f}° 已存'
        self.save_calib()
        return len(self.ranges)

    def sign(self, mid):
        return self.signs.get(mid, 1)

    def flip_sign(self, mid):
        previous = self.signs.copy()
        self.signs[mid] = -self.sign(mid)
        try:
            self.save_calib()
        except Exception:
            self.signs = previous
            raise
        return self.signs[mid]

    def to_policy(self, angles):
        """上位机角度 → 策略坐标系角度（乘符号）。"""
        return {m: round(self.sign(m) * d, 2) for m, d in angles.items()}

    def connect(self):
        if self.connected:
            return True
        self.last_error = ''
        self.calib, self.signs, self.ranges = {}, {}, {}
        self.motion_blocked = False
        ids = self.ids or DEFAULT_IDS
        if self.sim:
            self.connected, self.conn_msg = True, '离线仿真'
        else:
            if su.Bus is None:
                self.conn_msg = f'motor_sweep 不可用: {su.BUS_IMPORT_ERR}'
                return False
            try:
                self.bus = su.Bus(self.port, self.baud)
            except Exception as e:          # noqa: BLE001
                msg = str(e)
                if 'denied' in msg or 'PermissionError' in msg or '拒绝' in msg:
                    self.conn_msg = f'串口 {self.port} 被别的程序占用（Windows 串口独占），先关掉那个程序'
                elif 'FileNotFoundError' in msg or 'could not open' in msg:
                    self.conn_msg = f'串口 {self.port} 不存在，用 --list-ports 看看'
                else:
                    self.conn_msg = f'串口打开失败: {e}'
                return False
            self.connected, self.conn_msg = True, f'{self.port} @ {self.baud}'
        # 不扫描过滤：14 个 ID 全建状态，离线的在界面上灰掉，方便一眼看出谁掉线
        self.states = {m: MotorState(spec_for(m)) for m in ids}
        self.load_calib()
        self.running = True
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()
        return True

    def shutdown(self):
        """调用方先停止播放器/探测任务；后台收尾后再允许下一次连接。"""
        self.measurement_stop.set()
        with self.measurement_lock:
            pass  # 等待实测卸力并落盘；GUI 通过后台任务调用，不阻塞主界面。
        self.range_rec = False
        if self._range_thread:
            self._range_thread.join()
            self._range_thread = None
        self.running = False
        if self.thread:
            self.thread.join()
            self.thread = None
        self.release_all()
        with self.io_lock:
            try:
                if self.bus:
                    for mid in self.states:
                        self._release(mid)
                    self.bus.close()
            finally:
                self.bus = None
                self.connected = False
                self.conn_msg = '未连接'
                with self.lock:
                    self.states = {m: MotorState(spec_for(m)) for m in (self.ids or DEFAULT_IDS)}

    # ---------- 零位持久化 ----------
    def load_calib(self):
        if not os.path.exists(CALIB_FILE):
            return 0
        try:
            with open(CALIB_FILE, encoding='utf-8-sig') as f:
                data = json.load(f)
            calib = {int(k): v for k, v in data.get('zeros', {}).items()}
            signs = {int(k): int(v) for k, v in data.get('signs', {}).items()}
            ranges = {int(k): [float(v[0]), float(v[1])] for k, v in data.get('ranges', {}).items()}
            if any(not 0 <= m < 15 or not math.isfinite(float(v['ex_zero'])) for m, v in calib.items()):
                raise ValueError('零位 ID 或角度无效')
            if any(not 0 <= m < 15 or v not in (-1, 1) for m, v in signs.items()):
                raise ValueError('方向符号无效')
            if any(not 0 <= m < 15 or not all(math.isfinite(x) for x in v) or v[0] >= v[1]
                   for m, v in ranges.items()):
                raise ValueError('行程无效')
        except Exception as e:              # noqa: BLE001
            self.calib, self.signs, self.ranges = {}, {}, {}
            self.last_error = f'读取零位文件失败：{e}；请重新标定'
            return 0
        self.calib, self.signs, self.ranges = calib, signs, ranges
        return sum(1 for mid, c in self.calib.items()
                   if mid in self.states and self._apply_zero(mid, c['ex_zero']))

    def _apply_zero(self, mid, ex_zero):
        """用保存的输出端绝对编码器零位，推算当前 outputPos 坐标系下的 center。"""
        fb = self._probe(mid)
        if not fb or fb.get('MError', 0):
            return False
        off = wrap_pi(fb.get('ExPos', fb['outputPos']) - ex_zero)
        with self.lock:
            st = self.states[mid]
            st.center = fb['outputPos'] - off
            st.cmd = st.target = off if st.enabled else 0.0
            st.centered = True
            st.fb = fb
            st.status = f'零位已载入 偏 {math.degrees(off):+.1f}°'
        return True

    def save_calib(self):
        data = dict(note='zeros: 输出端绝对编码器零位（ExPos，弧度，单圈），断电重开有效；'
                         'signs: 策略坐标系相对上位机坐标系的符号，+1 同向 -1 反向，实测后填',
                    zeros={str(k): v for k, v in sorted(self.calib.items())},
                    signs={str(k): v for k, v in sorted(self.signs.items())},
                    ranges={str(k): v for k, v in sorted(self.ranges.items())})
        atomic_json(CALIB_FILE, data)

    def zero_now(self, mid):
        """静止三次有效反馈标定本机零位；只保存文件，不改电机 Flash。"""
        if not self.connected or self.states[mid].enabled or self.range_rec or self.measurement_active:
            self.states[mid].status = '请先停止采集并卸力'
            return False
        samples = []
        for _ in range(3):
            fb = self._probe(mid)
            if not fb or fb.get('MError', 0):
                self.states[mid].status = '标定失败：反馈缺失或电机故障'
                return False
            samples.append(fb)
            time.sleep(0.04)
        qs = [fb['outputPos'] for fb in samples]
        if math.degrees(max(qs) - min(qs)) > STILL_DEG:
            with self.lock:
                self.states[mid].status = f'没静止（抖 {math.degrees(max(qs) - min(qs)):.1f}°），未标零'
            return False
        fb = sorted(samples, key=lambda f: f['outputPos'])[1]
        ex = fb.get('ExPos', fb['outputPos']) % TWO_PI
        previous = self.calib.copy()
        self.calib[mid] = dict(ex_zero=ex, when=datetime.now().strftime('%Y-%m-%d %H:%M:%S'))
        try:
            self.save_calib()
        except Exception:
            self.calib = previous
            raise
        with self.lock:
            st = self.states[mid]
            st.center = fb['outputPos']
            st.cmd = st.target = 0.0
            st.centered, st.fb = True, fb
            st.status = '零位已保存到本机'
        return True

    # ---------- 读角 / 走位姿 ----------
    def read_angles(self, use_actual):
        """{mid: 相对零位的角度(度)}，在线的台全记。use_actual 或未使能：读实际位置；否则读指令角。"""
        out = {}
        with self.lock:
            for mid, st in self.states.items():
                if not st.fb:
                    continue
                if use_actual or not st.enabled:
                    out[mid] = round(math.degrees(st.fb['outputPos'] - st.center), 2)
                else:
                    out[mid] = round(math.degrees(st.cmd), 2)
        return out

    def unzeroed(self):
        with self.lock:
            return [m for m, st in self.states.items() if st.fb and not st.centered]

    def _enable_hold(self, mids):
        """把这些台使能并锁在实际位置（不下目标）。"""
        with self.lock:
            for m in mids:
                st = self.states.get(m)
                if self.measurement_active or self.range_rec or self.motion_blocked or not self.connected or st is None or not st.centered or st.error or st.enabled or not st.fb:
                    continue
                st.cmd = st.target = st.fb['outputPos'] - st.center
                st.enabled = True
                st.status = '已使能'

    def goto(self, angles):
        """整机按全局限速走到一组角度。未标零或有告警的跳过。"""
        angles = {m: self.validate_target(m, d) for m, d in angles.items()}
        self._enable_hold(angles)
        for mid, deg in angles.items():
            st = self.states.get(mid)
            if st is None or not st.enabled:
                continue
            self.set_target(mid, deg)

    def all_arrived(self, tol=0.5):
        with self.lock:
            return all(not st.enabled or abs(math.degrees(st.target - st.cmd)) <= tol
                       for st in self.states.values())

    def transition(self, target, duration, stop_evt, peak_limit=None):
        """从当前位置 minjerk 过渡到 target（度），直接写 cmd 绕过全局限速。
        峰值速度 = 1.875 × 行程 / 时长，超过 peak_limit 自动拉长。返回实际用时。"""
        if stop_evt.is_set():
            return 0.0
        follow_speed = peak_limit is None
        peak_limit = self.speed if follow_speed else min(peak_limit, SPEED_MAX)
        if not math.isfinite(peak_limit) or peak_limit < 0:
            raise ValueError('平滑过渡速度无效')
        if peak_limit == 0:
            return 0.0
        target = {m: self.validate_target(m, d) for m, d in target.items()}
        self._enable_hold(target)
        time.sleep(su.DT * 2)
        start = self.read_angles(use_actual=False)
        ids = [m for m in target if m in self.states and self.states[m].enabled and not self.states[m].error]
        if not ids:
            return 0.0
        span = max(abs(target[m] - start.get(m, target[m])) for m in ids)
        duration = max(float(duration), 1.875 * span / peak_limit, su.DT)
        t0 = time.time(); previous = t0; progress_s = 0.0
        while not stop_evt.is_set():
            now = time.time()
            # 实时调速只改变曲线进度；0 暂停，恢复后连续推进，不跳目标。
            scale = min(1.0, self.speed / peak_limit) if follow_speed else 1.0
            progress_s += (now - previous) * scale; previous = now
            s = progress_s / duration
            frame = interp(start, target, min(1.0, s))
            with self.lock:
                for m in ids:
                    st = self.states[m]
                    if not st.enabled or st.error:
                        continue
                    st.target = st.cmd = math.radians(frame[m])
            if s >= 1.0:
                break
            time.sleep(su.DT)
        return duration

    def refresh_offline(self):
        """未使能的台也定期探测，让在线/离线和实际角度能显示。"""
        for mid, st in list(self.states.items()):
            if not self.connected or self.measurement_active:
                return
            if st.enabled:
                continue
            fb = self._probe(mid)
            with self.lock:
                if fb:
                    st.fb, st.comm_fail = fb, 0
                else:
                    st.comm_fail += 1
                    if st.comm_fail >= 3:
                        st.fb = None
            if fb and not st.centered and mid in self.calib and self.connected:
                self._apply_zero(mid, self.calib[mid]['ex_zero'])


    def reserve_measurement(self, config):
        """GUI线程预留互斥，随后在维护线程 collect；不执行任何运动。"""
        config.validate()
        with self.lock:
            if not self.connected or self.sim or not self.bus or self.measurement_active or self.range_rec:
                raise ValueError('需连接真实串口、停止行程采集且没有其他实测任务')
            if any(s.enabled for s in self.states.values()):
                raise ValueError('先全部卸力并固定机器人/台架')
            if any(m not in self.states for m in config.ids):
                raise ValueError('所选ID不在本次连接集合中')
            if config.kind != 'latency' and any(not self.states[m].centered or self.states[m].error for m in config.ids):
                raise ValueError('主动采集必须已标定且无故障')
            self.measurement_stop.clear()
            self.measurement_active = True
            self.motion_blocked = True

    def collect_measurement(self, config, output_dir):
        from measurement import MeasurementRunner
        with self.measurement_lock:
            if not self.measurement_active:
                raise ValueError('未预留采集会话')
            try:
                self.running = False
                if self.thread:
                    self.thread.join(); self.thread = None
                with self.io_lock:
                    # 清理普通控制的待卸力请求；不对选中台提前发额外读写污染统计。
                    for m, st in self.states.items():
                        if st.want_release:
                            self._release(m); st.want_release = False
                self.measurement_runner = MeasurementRunner(self.bus, self.io_lock, output_dir, self.measurement_stop)
                centers = {m:self.states[m].center for m in config.ids if self.states[m].centered}
                metadata = dict(port=self.port, baud=self.baud, simulated=False,
                                joints={str(m):self.states[m].spec for m in config.ids},
                                signs=self.signs, calibration=self.calib,
                                sign_note='默认+1不代表实测确认；摆锤direction独立于模型direction')
                folder, result = self.measurement_runner.run(config, centers=centers, metadata=metadata)
                return f'{result["status"]}：{folder}；有效样本 {result["summary"]["valid"]}/{result["summary"]["samples"]}；' + '；'.join(result['errors'])
            finally:
                with self.lock:
                    for st in self.states.values():
                        st.enabled = False; st.target = st.cmd
                        st.fb = None; st.status = '实测结束，未使能；等待新反馈'
                    self.measurement_active = False
                if self.connected:
                    self.running = True
                    self.thread = threading.Thread(target=self._loop, daemon=True); self.thread.start()

    def export_calibration(self, path):
        """导出当前主机标定副本；不推定q_home/方向已验证。"""
        if self.measurement_active:
            raise ValueError('请先停止实测')
        atomic_json(path, dict(zeros={str(k):v for k,v in self.calib.items()},
                               signs={str(k):v for k,v in self.signs.items()},
                               ranges={str(k):v for k,v in self.ranges.items()},
                               note='电脑端标定副本，不是电机Flash或模型q_home；未显式确认的方向不得默认视作实测'))


class PoseLibrary:
    """命名位姿列表与播放。播放在后台线程，进度通过 msg 字段轮询。"""

    def __init__(self, ctrl):
        self.ctrl = ctrl
        self.poses = []                     # [dict(name, angles{mid: deg})]
        self.thread = None
        self.stop = threading.Event()
        self.msg = ''
        self.load_warning = ''

    # ---- 编辑 ----
    def record(self, name, use_actual, speed=None, dwell=1.0):
        ang = self.ctrl.read_angles(use_actual)
        if not ang:
            return None
        name = name.strip() or f'pose{len(self.poses) + 1}'
        speed = self.ctrl.speed if speed is None else float(speed)
        self.validate_motion(speed, dwell)
        self.poses.append(dict(name=name, angles=ang, speed=speed, dwell=float(dwell)))
        return self.poses[-1]

    def set_timing(self, i, speed=None, dwell=None):
        # 旧 JSON 仅在 load 中显式迁移，禁止静默忽略旧 trans 调用。
        self.validate_motion(self.poses[i].get('speed', SPEED_DEFAULT) if speed is None else speed,
                             self.poses[i].get('dwell', 1.0) if dwell is None else dwell)
        if speed is not None:
            self.poses[i]['speed'] = float(speed)
        if dwell is not None:
            self.poses[i]['dwell'] = float(dwell)

    @staticmethod
    def validate_motion(speed, dwell):
        if not math.isfinite(float(speed)) or not 0 < float(speed) <= SPEED_MAX:
            raise ValueError('动作速度须大于0且不超过协议上界；暂停请填写动作结束暂停时间')
        if not math.isfinite(float(dwell)) or float(dwell) < 0:
            raise ValueError('动作结束暂停须为非负有限秒数')

    def set_angle(self, i, mid, deg):
        """手工编辑保留角度，仅校验数据和协议可编码性。"""
        self.poses[i]['angles'][mid] = round(self.ctrl.validate_target(mid, deg), 2)

    def overwrite(self, i, use_actual):
        self.poses[i]['angles'] = self.ctrl.read_angles(use_actual)

    def delete(self, i):
        del self.poses[i]

    def move(self, i, d):
        j = i + d
        if 0 <= j < len(self.poses):
            self.poses[i], self.poses[j] = self.poses[j], self.poses[i]
            return j
        return i

    def insert_mid(self, i, n):
        """在 i 和 i+1 之间按 minjerk 进度插 n 个中间位姿。"""
        if i + 1 >= len(self.poses):
            return 0
        a, b = self.poses[i], self.poses[i + 1]
        speed = b.get('speed', self.ctrl.speed)
        mids = [dict(name=f'{a["name"]}→{b["name"]}_{k}', angles=interp(a['angles'], b['angles'], k / (n + 1)),
                     speed=speed, dwell=0.0)
                for k in range(1, n + 1)]
        self.poses[i + 1:i + 1] = mids
        return n

    # ---- 存取 ----
    def save(self, path=POSE_FILE):
        data = dict(note='相对零位的角度（度），键为舵机 ID；配合 calib_host_xianduck.json 使用',
                    saved=datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
                    signs={str(k): v for k, v in sorted(self.ctrl.signs.items())},
                    poses=[dict(name=p['name'], speed=p.get('speed', SPEED_DEFAULT), dwell=p.get('dwell', 1.0),
                                legacy_trans_s=p.get('legacy_trans_s', p.get('trans')),
                                angles={str(k): v for k, v in p['angles'].items()},
                                policy={str(k): v for k, v in self.ctrl.to_policy(p['angles']).items()})
                           for p in self.poses])
        data.update(schema_version=3, speed_unit='deg/s', pause_unit='s', angle_unit='deg', coordinate_frame='host_zero_relative',
                    joint_order=list(self.ctrl.states),
                    joint_order_kind='ui_display_order_not_policy_order', angle_limits_enabled=False,
                    joints={str(m): dict(name=s.spec['name'], joint_name=s.spec['joint'], group=s.spec['group'], verified=s.spec['verified'])
                            for m, s in self.ctrl.states.items()},
                    calibration_file=os.path.basename(CALIB_FILE))
        atomic_json(path, data)

    def export_csv(self, path):
        """一帧一行；角度按明确 ID 列排列。遥测 CSV 由界面单独记录。"""
        mids = list(self.ctrl.states)
        with open(path, 'w', newline='', encoding='utf-8-sig') as f:
            writer = csv.writer(f)
            writer.writerow(['name', 'speed_deg_s', 'dwell_s'] + [f'id_{m}_deg' for m in mids])
            for p in self.poses:
                writer.writerow([p['name'], p.get('speed', SPEED_DEFAULT), p.get('dwell', 1.0)] +
                                [p['angles'].get(m, '') for m in mids])

    def load(self, path=POSE_FILE):
        with open(path, encoding='utf-8-sig') as f:
            data = json.load(f)
        if data.get('schema_version', 1) not in (1, 2, 3):
            raise ValueError('不支持的位姿格式版本')
        if data.get('angle_unit', 'deg') != 'deg' or data.get('coordinate_frame', 'host_zero_relative') != 'host_zero_relative':
            raise ValueError('角度必须为相对本机零位的度数')
        if data.get('speed_unit', 'deg/s') != 'deg/s' or data.get('pause_unit', 's') != 's':
            raise ValueError('速度单位必须为deg/s，暂停单位必须为s')
        poses = []; migrated = 0
        for i, p in enumerate(data['poses']):
            name = p['name']
            speed = float(p.get('speed', SPEED_DEFAULT))
            dwell = float(p.get('dwell', 1.0))
            if 'speed' not in p:
                migrated += 1
            angles = {int(k): float(v) for k, v in p['angles'].items()}
            if not isinstance(name, str) or not name.strip() or not angles:
                raise ValueError(f'第 {i + 1} 帧名称或角度为空')
            self.validate_motion(speed, dwell)
            for mid, deg in angles.items():
                if mid not in self.ctrl.states:
                    raise ValueError(f'第 {i + 1} 帧含未加载电机 ID {mid}')
                self.ctrl.validate_target(mid, deg)
            legacy = p.get('legacy_trans_s', p.get('trans'))
            if legacy is not None and (not math.isfinite(float(legacy)) or float(legacy) <= 0):
                raise ValueError('旧过渡时长无效')
            poses.append(dict(name=name, speed=speed, dwell=dwell, angles=angles, legacy_trans_s=legacy))
        self.poses = poses  # 全部校验通过后才替换，失败保留当前库。
        self.load_warning = (f'旧文件 {migrated} 帧缺少运动速度，已按 {SPEED_DEFAULT}°/s 迁移；原过渡秒数保留为 legacy_trans_s，不再控制播放，请逐帧确认。' if migrated else '')

    def halt(self):
        """停止播放并等待最后一个动作退出，供断开/退出的后台任务使用。"""
        self.stop.set()
        if self.thread:
            self.thread.join()

    # ---- 播放 ----
    def playing(self):
        return bool(self.thread and self.thread.is_alive())

    def _start(self, fn):
        if self.playing():
            return False
        self.stop.clear()
        def guarded():
            try:
                fn()
            except Exception as e:
                self.ctrl.release_all()
                self.msg = f'播放失败，已卸力：{e}'
        self.thread = threading.Thread(target=guarded, daemon=True)
        self.thread.start()
        return True

    def _execute_pose(self, p, smooth=False):
        speed, dwell = p.get('speed', SPEED_DEFAULT), p.get('dwell', 1.0)
        self.validate_motion(speed, dwell)
        if self.stop.is_set():
            return False
        ids = list(p['angles'])
        with self.ctrl.lock:
            if not self.ctrl.connected or self.ctrl.motion_blocked:
                raise ValueError('控制已断开或急停锁存')
            for mid in ids:
                st = self.ctrl.states[mid]
                if not st.centered or not st.fb or st.error:
                    raise ValueError(f'ID {mid} 未标定、离线或故障')
        self.ctrl.set_speed(speed)
        start = self.ctrl.read_angles(use_actual=True)
        span = max(abs(p['angles'][m]-start[m]) for m in ids)
        deadline = time.monotonic() + max(3.0, span/speed*(1.875 if smooth else 1)+3.0)
        self.msg = f'执行 {p["name"]}：{speed:g}°/s'
        if smooth:
            self.ctrl.transition(p['angles'], .1, self.stop, peak_limit=speed)
        else:
            self.ctrl.goto(p['angles'])
        stable = 0
        observed = {}
        pause_until = None
        while not self.stop.is_set():
            with self.ctrl.lock:
                if not self.ctrl.connected or self.ctrl.motion_blocked:
                    raise RuntimeError('控制已断开或急停锁存，停止播放')
                arrived = True
                feedback = {}
                for mid in ids:
                    st = self.ctrl.states[mid]
                    if not st.enabled or st.error or not st.fb or st.comm_fail:
                        raise RuntimeError(f'ID {mid} 无有效反馈、未使能或故障，停止播放')
                    arrived &= (abs(math.degrees(st.cmd)-p['angles'][mid]) <= .5 and
                                abs(math.degrees(st.fb['outputPos']-st.center)-p['angles'][mid]) <= .5)
                    feedback[mid] = st.fb
            # 控制线程每次读取都替换反馈对象；同一份反馈不能重复计为到位。
            if not arrived:
                stable = 0
            elif all(feedback[m] is not observed.get(m) for m in ids):
                stable += 1
                observed = feedback
            now = time.monotonic()
            if pause_until is not None:
                if now >= pause_until:
                    return True
            elif stable >= 3:
                self.msg = f'{p["name"]} 已到位，暂停 {dwell:g}s'
                pause_until = now + dwell
            elif now >= deadline:
                raise RuntimeError(f'{p["name"]} 未在预计时间内到位，停止播放')
            self.stop.wait(.02)
        return False

    def goto_smooth(self, i, smooth=False):
        p = dict(self.poses[i])
        def run():
            previous = self.ctrl.speed
            try:
                finished = self._execute_pose(p, smooth)
                self.msg = '动作完成' if finished else '已停止'
            finally:
                self.ctrl.set_speed(previous)
        return self._start(run)

    def play(self, repeat, smooth=False):
        """逐帧按自己的速度执行，反馈到位后开始暂停；任何失败终止序列。"""
        if not self.poses:
            return False
        def run():
            previous = self.ctrl.speed
            done = 0
            try:
                while not self.stop.is_set():
                    done += 1
                    for p in list(self.poses):
                        if not self._execute_pose(p, smooth):
                            self.msg = '已停止'
                            return
                    if repeat and done >= repeat:
                        break
                self.msg = '已停止' if self.stop.is_set() else f'播放结束，共 {done} 遍'
            finally:
                self.ctrl.set_speed(previous)
        return self._start(run)
