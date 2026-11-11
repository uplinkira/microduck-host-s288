#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
J288/S288 单总线底层收发 + 测试记录

协议部分改写自官方 demo:
  ../docs/datasheets/舵机demo.zip → Python 通信说明/J288-S288__PythonDemo.py
协议定义见手册第九节: ../docs/datasheets/无刷数字舵机J288S288使用手册.pdf

对外接口(servo_ui.py / motor_random.py 依赖):
  Bus(port, baud)
    .transact(mid, status, tor, spd, pos, kp, kd) -> dict|None   发控制包并读反馈
    .probe(mid)                                   -> dict|None   零力矩探测,只读状态不出力
    .release(mid, kd=0.3)                                        卸力(Kp=0,留少量阻尼)
    .close()
  MotorTest(bus, csv_path)
    .log(mid, phase, q_cmd, fb) -> fb   记一行并更新峰值统计
    .save_csv()
    .t0 / .peak_torque / .max_temp / .min_volt
  TEMP_ABORT / TEMP_WARN / VOLT_WARN / TORQUE_WARN

量纲约定: 对外的 tor/spd/pos/kp/kd 全部是**输出端**物理量(N·m, rad/s, rad),
内部按减速比 RATIO 折算到转子端再打包 —— 与官方 demo 一致。
"""
import sys, os, time, math, csv, struct, threading
if hasattr(sys.stdout, 'reconfigure'):          # Windows 控制台默认 GBK，打印 ✓ ✗ 会报错
    sys.stdout.reconfigure(encoding='utf-8', errors='replace')

# pyserial 只在真正开串口时才需要,--selftest 和纯协议调用不依赖它
serial = None


def _need_serial():
    global serial
    if serial is None:
        try:
            import serial as _s
        except ImportError:
            raise SystemExit('✗ 缺少 pyserial。开发板上执行:\n'
                             '    sudo apt install -y python3-serial\n'
                             '  或 pip3 install pyserial')
        serial = _s
    return serial

# ==================== 协议常量 ====================
RATIO = 70070.0 / 243.0          # 减速比 288.3539...
M_PI = math.pi

CMD_LEN, FB_LEN = 20, 26
HEAD_CMD = b'\xFE\xEE'
HEAD_FB = b'\xFC\xEE'

# 输出端限幅(官方 demo 取值)
KP_MAX, KD_MAX = 2128.523254, 21.285233
TOR_MAX, SPD_MAX, POS_MAX = 36.908174, 278.90143, 1428.018898

# 保护阈值(见 仓库 README 与本文件参数表)
TEMP_ABORT = 85      # 壳温 °C,达到即停机
TEMP_WARN = 70       # 壳温 °C,告警
VOLT_WARN = 11.0     # 端电压 V,低于即告警
TORQUE_WARN = 3.0    # 输出端扭矩 N·m,超过即告警(可能卡滞)


def _make_crc_table():
    """CRC32 查表,多项式 0x04C11DB7,MSB-first。已验证与官方 demo 的硬编码表逐项一致。"""
    table = []
    for i in range(256):
        c = i << 24
        for _ in range(8):
            c = ((c << 1) ^ 0x04C11DB7) & 0xFFFFFFFF if c & 0x80000000 else (c << 1) & 0xFFFFFFFF
        table.append(c)
    return table


CRC32_TABLE = _make_crc_table()


def crc32_core(data):
    """按 32 位字处理(每 4 字节逆序喂入),对应手册里操作 uint32_t* 的 C 实现。"""
    crc = 0xFFFFFFFF
    for i in range(0, len(data) - 3, 4):
        for b in (data[i + 3], data[i + 2], data[i + 1], data[i]):
            crc = CRC32_TABLE[(crc >> 24) ^ b] ^ ((crc << 8) & 0xFFFFFFFF)
    return crc


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


def build_control_packet(mid, mode, timeout, tor, spd, pos, kp, kd):
    """输出端物理量 → 20 字节控制包。"""
    mid = _clamp(int(mid), 0, 15)
    mode = _clamp(int(mode), 0, 1)
    timeout = 1 if timeout else 0
    kp = _clamp(float(kp), 0.0, KP_MAX)
    kd = _clamp(float(kd), 0.0, KD_MAX)
    tor = _clamp(float(tor), -TOR_MAX, TOR_MAX)
    spd = _clamp(float(spd), -SPD_MAX, SPD_MAX)
    pos = _clamp(float(pos), -POS_MAX, POS_MAX)

    mode_byte = (mid & 0x0F) | ((mode & 0x07) << 4) | ((timeout & 0x01) << 7)

    # 输出端 → 转子端: 刚度/阻尼过 RATIO²,位置/速度乘 RATIO,扭矩除 RATIO
    k_pos = int(round(kp / (RATIO * RATIO) * 1280000.0))
    k_spd = int(round(kd / (RATIO * RATIO) * 128000000.0))
    pos_des = int(round(pos * RATIO * 32768.0 / M_PI / 2.0))
    spd_des = int(round(spd * RATIO * 2.560 / M_PI / 2.0))
    tor_des = int(round(tor / RATIO * 256000.0))

    k_pos = _clamp(k_pos, -32768, 32767)
    k_spd = _clamp(k_spd, -32768, 32767)
    spd_des = _clamp(spd_des, -32768, 32767)
    tor_des = _clamp(tor_des, -32768, 32767)
    pos_des = _clamp(pos_des, -2147483648, 2147483647)


    body = struct.pack('<hhihh', tor_des, spd_des, pos_des, k_pos, k_spd)
    head = HEAD_CMD + bytes([mode_byte]) + b'\x00'      # 第 4 字节为保留位
    pkt = head + body
    pkt += struct.pack('<I', crc32_core(pkt))           # 帧头参与 CRC
    assert len(pkt) == CMD_LEN, len(pkt)
    return pkt


def parse_feedback_packet(data):
    """26 字节反馈 → dict(输出端物理量)。校验失败返回 None。"""

    if len(data) != FB_LEN or data[0] != 0xFC or data[1] != 0xEE:
        return None
    # 反馈的帧头不参与 CRC: 校验范围是 data[2:22],共 20 字节
    if struct.unpack('<I', data[22:26])[0] != crc32_core(data[2:22]):
        return None

    mode_byte = data[2]
    temp, sensor, vol, torque, speed, pos, merror, outpos_exflag, _res1, _res2 = \
        struct.unpack('<bbb hh i I H BB', data[3:22])

    return {
        'motor_id': mode_byte & 0x0F,
        'mode': (mode_byte >> 4) & 0x07,
        'timeout': (mode_byte >> 7) & 0x01,
        'Temp': temp,                                   # 壳体温度 °C
        'sensor': sensor,                               # 绕组温度 °C
        'vol': vol / 2.0,                               # 端电压 V
        'MError': merror,                               # 故障码,见手册第十节
        'MWarn': (outpos_exflag >> 13) & 0x07,          # 警告码
        'ExPos': 2 * M_PI * (outpos_exflag & 0x1FFF) / 8192.0,   # 输出端编码器,13bit/圈
        'outputTor': (torque / 256000.0) * RATIO,       # 输出端扭矩 N·m
        'outputSpd': (speed / 2.56) * 2 * M_PI / RATIO,  # 输出端速度 rad/s
        'outputPos': 2 * M_PI * pos / 32768.0 / RATIO,   # 输出端位置 rad
    }


class Bus:
    """单总线半双工收发。总线同一时刻只能一个方向通信,故整体加锁串行化。"""

    def __init__(self, port, baud=6000000, timeout=0.02):
        self.ser = _need_serial().Serial(port, baud, timeout=timeout)
        self.lock = threading.Lock()
        self.tx = self.rx = self.bad = 0

    def transact(self, mid, status=1, tor=0.0, spd=0.0, pos=0.0, kp=0.0, kd=0.0,
                 timeout_bit=0):
        """发一包读一包。timeout_bit 默认 0(禁用舵机 1s 超时保护),与项目安全策略一致。

        注意 timeout_bit=0 意味着进程死掉后电机会一直顶住最后指令,
        所有退出路径必须显式 release() —— 见 仓库 README 与本文件参数表。
        """
        pkt = build_control_packet(mid, status, timeout_bit, tor, spd, pos, kp, kd)
        with self.lock:
            try:
                self.ser.reset_input_buffer()    # 丢弃上一轮的残包,避免错帧
                self.ser.write(pkt)
                self.tx += 1
                fb = self.ser.read(FB_LEN)
            except Exception:                 # 串口异常(拔线/权限/超时)一律按丢包处理
                self.bad += 1
                return None
        if len(fb) != FB_LEN:
            self.bad += 1
            return None
        parsed = parse_feedback_packet(fb)
        if parsed is None:
            self.bad += 1
            return None
        self.rx += 1
        return parsed

    def probe(self, mid):
        """零力矩探测: Kp=Kd=0,只读状态不出力。"""
        return self.transact(mid, status=1, tor=0.0, spd=0.0, pos=0.0, kp=0.0, kd=0.0)

    def release(self, mid, kd=0.3):
        """卸力: Kp=0,保留少量阻尼避免关节自由落体。重发 3 次提高送达率。"""
        last = None
        for _ in range(3):
            last = self.transact(mid, status=1, tor=0.0, spd=0.0, pos=0.0, kp=0.0, kd=kd)
            time.sleep(0.002)
        return last

    def close(self):
        with self.lock:
            if self.ser.is_open:
                self.ser.close()

    def stats(self):
        return f'tx={self.tx} rx={self.rx} bad={self.bad}'


class MotorTest:
    """采样记录 + 峰值统计,收尾写 CSV。"""

    FIELDS = ['t', 'id', 'phase', 'q_cmd_deg', 'pos_deg', 'spd_dps',
              'torque', 'temp', 'sensor', 'vol', 'MError', 'MWarn']

    def __init__(self, bus, csv_path='/tmp/motor_log.csv'):
        self.bus = bus
        self.csv_path = csv_path
        self.rows = []
        self.t0 = time.time()
        self.peak_torque = 0.0
        self.max_temp = -128
        self.min_volt = 1e9

    def log(self, mid, phase, q_cmd, fb):
        if not fb:
            return fb
        self.peak_torque = max(self.peak_torque, abs(fb['outputTor']))
        self.max_temp = max(self.max_temp, fb['Temp'])
        self.min_volt = min(self.min_volt, fb['vol'])
        self.rows.append({
            't': round(time.time() - self.t0, 4),
            'id': mid, 'phase': phase,
            'q_cmd_deg': round(math.degrees(q_cmd), 3),
            'pos_deg': round(math.degrees(fb['outputPos']), 3),
            'spd_dps': round(math.degrees(fb['outputSpd']), 2),
            'torque': round(fb['outputTor'], 4),
            'temp': fb['Temp'], 'sensor': fb['sensor'],
            'vol': round(fb['vol'], 2),
            'MError': fb['MError'], 'MWarn': fb['MWarn'],
        })
        return fb

    def save_csv(self):
        if not self.rows:
            print('无采样数据,不写 CSV')
            return
        try:
            os.makedirs(os.path.dirname(os.path.abspath(self.csv_path)), exist_ok=True)
            with open(self.csv_path, 'w', newline='', encoding='utf-8') as f:
                w = csv.DictWriter(f, fieldnames=self.FIELDS)
                w.writeheader()
                w.writerows(self.rows)
            print(f'已写入 {len(self.rows)} 行 → {self.csv_path}')
        except OSError as e:
            print(f'写 CSV 失败: {e}')


# ==================== 自检 / 扫描 ====================
def main():
    """python3 motor_sweep.py [--port X] [--selftest]

    --selftest  不连硬件,只跑打包/解析的往返一致性检查
    默认        扫描总线上 ID 0~14,打印在线电机状态
    """
    import argparse
    ap = argparse.ArgumentParser(description='J288/S288 总线扫描 / 协议自检')
    ap.add_argument('--port', default='/dev/ttyACM0')
    ap.add_argument('--baud', type=int, default=6000000)
    ap.add_argument('--selftest', action='store_true', help='离线协议自检,不开串口')
    a = ap.parse_args()

    if a.selftest:
        ok = True
        # 1. 控制包长度与帧头
        pkt = build_control_packet(7, 1, 0, 0.0, 0.0, math.radians(30), 10.0, 0.5)
        ok &= len(pkt) == CMD_LEN and pkt[:2] == HEAD_CMD
        print(f'控制包 20 字节 / 帧头正确: {len(pkt) == CMD_LEN and pkt[:2] == HEAD_CMD}')
        print(f'  {pkt.hex(" ")}')
        # 2. 自造一个反馈包,验证解析能还原
        body = struct.pack('<bbb hh i I H BB', 42, 55, 48,
                           int(0.5 / RATIO * 256000), 0,
                           int(math.radians(30) * RATIO * 32768 / M_PI / 2), 0,
                           1234, 0, 0)
        raw = HEAD_FB + bytes([7 | (1 << 4)]) + body
        raw += struct.pack('<I', crc32_core(raw[2:]))
        fb = parse_feedback_packet(raw)
        ok &= fb is not None
        if fb:
            got, want = math.degrees(fb['outputPos']), 30.0
            ok &= abs(got - want) < 0.05 and abs(fb['outputTor'] - 0.5) < 0.01
            ok &= fb['Temp'] == 42 and fb['vol'] == 24.0 and fb['motor_id'] == 7
            print(f'反馈解析: id={fb["motor_id"]} 位置={got:.3f}° (期望 30) '
                  f'扭矩={fb["outputTor"]:.3f}N·m (期望 0.5) 壳温={fb["Temp"]}°C 电压={fb["vol"]}V')
        # 3. CRC 坏包必须被拒
        bad = bytearray(raw); bad[10] ^= 0xFF
        ok &= parse_feedback_packet(bytes(bad)) is None
        print(f'坏 CRC 被拒: {parse_feedback_packet(bytes(bad)) is None}')
        print(f'\n{"✓ 自检通过" if ok else "✗ 自检失败"}')
        return 0 if ok else 1

    bus = Bus(a.port, a.baud)
    print(f'扫描 {a.port} @ {a.baud} bps,ID 0~14 ...')
    found = 0
    for mid in range(15):
        fb = bus.probe(mid)
        if fb:
            found += 1
            print(f'  ID {mid:2d}: 位置 {math.degrees(fb["outputPos"]):+8.2f}°  '
                  f'壳温 {fb["Temp"]:3d}°C  绕组 {fb["sensor"]:3d}°C  '
                  f'{fb["vol"]:5.1f}V  扭矩 {fb["outputTor"]:+6.2f}N·m  '
                  f'MError=0x{fb["MError"]:X}')
    print(f'共发现 {found} 台 | {bus.stats()}')
    bus.close()
    return 0


if __name__ == '__main__':
    sys.exit(main())
