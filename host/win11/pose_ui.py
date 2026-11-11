#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
标零与位姿记录上位机（PyQt6 界面；控制逻辑在 pose_core.py）

  python pose_ui.py                       # 主窗口内选择串口并连接，ID 1~14
  python pose_ui.py --port COM5
  python pose_ui.py --ids 1,2,3,4,5       # 只带这些 ID
  python pose_ui.py --sim                 # 离线仿真，不连硬件
  python pose_ui.py --list-ports

打包：build.bat（PyInstaller 单文件）。
"""
import sys, os, math, argparse, threading

if hasattr(sys.stdout, 'reconfigure'):
    sys.stdout.reconfigure(encoding='utf-8', errors='replace')
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtGui import QFont, QColor, QShortcut, QKeySequence, QFontDatabase
from PyQt6.QtWidgets import (QApplication, QMainWindow, QWidget, QLabel, QPushButton, QCheckBox,
                             QSlider, QDoubleSpinBox, QSpinBox, QLineEdit, QRadioButton, QGroupBox, QComboBox,
                             QGridLayout, QHBoxLayout, QVBoxLayout, QFrame, QTableWidget,
                             QTableWidgetItem, QHeaderView, QMessageBox, QFileDialog, QAbstractItemView, QScrollArea)

import pose_core as pc
from pose_core import PoseController, PoseLibrary, list_ports
from measurement_ui import MeasurementDialog
from mapping_ui import MappingDialog
from workbench_ui import WorkbenchDialog
from robot_profile import draft_from_hardware

MONO = QFont('Consolas', 9)
GREEN, GREY, RED, ORANGE, BLUE = '#2a2', '#bbb', '#b00', '#c60', '#06c'


def configure_font(app):
    """中文采用系统 UI 字体，等宽字体仅用于纯数值。"""
    available = set(QFontDatabase.families())
    families = [f for f in ('Microsoft YaHei UI', 'Microsoft YaHei', 'Noto Sans CJK SC', 'SimHei') if f in available]
    font = QFont()
    if families:
        font.setFamilies(families)
    font.setPointSize(9)
    app.setFont(font)


def hline():
    f = QFrame(); f.setFrameShape(QFrame.Shape.HLine); f.setFrameShadow(QFrame.Shadow.Sunken)
    return f


def lbl(text='', w=None, mono=False, align=Qt.AlignmentFlag.AlignRight, color=None):
    q = QLabel(text)
    if mono:
        q.setFont(MONO)
    if w:
        q.setMinimumWidth(max(w, q.fontMetrics().horizontalAdvance(text) + 8))
    q.setAlignment(align | Qt.AlignmentFlag.AlignVCenter)
    if color:
        q.setStyleSheet(f'color:{color}')
    return q


def btn(text, fn, w=None):
    b = QPushButton(text); b.clicked.connect(fn)
    if w:
        b.setMinimumWidth(max(w, b.fontMetrics().horizontalAdvance(text) + 24))
    return b


class Row:
    """一台舵机一行，所有控件横向排在 QGridLayout 的同一行。"""
    HEADS = ['ID', '关节名称', '在线', '使能', '标定', '零位 (°)', '方向', '目标角（相对零位）', '目标 (°)', '实际 (°)',
             '偏差 (°)', '温度 (℃)', '力矩 (N·m)', '电压 (V)', '状态', '', '', '映射状态']

    def __init__(self, grid, r, ctrl, spec, zero_fn):
        self.ctrl, self.mid, self.spec = ctrl, spec['id'], spec
        self._sync = False
        lim = 180  # 编辑视野，可独立缩放；不代表机械行程。
        self.w_id = lbl(f'{self.mid:2d}', 24, mono=True)
        self.w_name = lbl(spec['name'], 100, align=Qt.AlignmentFlag.AlignLeft)
        self.w_verified = lbl('已确认' if spec['verified'] else '待确认', color=GREEN if spec['verified'] else ORANGE)
        self.w_name.setToolTip(f'ID {self.mid} · {spec["group"]} · {spec["name"]}\n' +
                              ('映射已实测确认' if spec['verified'] else '依据装配图推定，需逐台实测确认；不是通信错误'))
        self.w_on = lbl('●', 16, color=GREY, align=Qt.AlignmentFlag.AlignCenter)
        self.w_en = QCheckBox(); self.w_en.clicked.connect(self._toggle)
        self.w_zero = btn('设零位', lambda: zero_fn(self.mid), 60)
        self.w_zero.setToolTip('卸力并保持静止后，将当前位置保存为本机零位；不写电机 Flash')
        self.w_zv = lbl('—', 46, mono=True)
        self.w_sg = btn('+', self._flip, 24); self.w_sg.setFont(MONO)
        self.w_sg.setToolTip('策略坐标系相对上位机的方向符号，实测后点一下翻转，存进零位文件')
        self.w_sl = QSlider(Qt.Orientation.Horizontal)
        self.w_sl.setRange(int(-lim * 2), int(lim * 2)); self.w_sl.setMinimumWidth(300)
        self.w_slider_panel = QWidget()
        slider_layout = QVBoxLayout(self.w_slider_panel)
        slider_layout.setContentsMargins(0, 0, 0, 0); slider_layout.setSpacing(0)
        slider_layout.addWidget(self.w_sl)
        range_bar = QHBoxLayout(); range_bar.setContentsMargins(0, 0, 0, 0)
        self.w_range = QLabel()
        self.w_expand = btn('扩大范围', lambda: self._zoom_slider(2))
        self.w_narrow = btn('缩小范围', lambda: self._zoom_slider(0.5))
        for button in (self.w_expand, self.w_narrow):
            button.setToolTip('只调整滑条显示范围，不发送运动指令；不是机械限位设置')
        range_bar.addWidget(self.w_range, 1)
        range_bar.addWidget(self.w_narrow); range_bar.addWidget(self.w_expand)
        slider_layout.addLayout(range_bar)
        self._update_range_label()
        self.w_sp = QDoubleSpinBox(); self.w_sp.setRange(-1e9, 1e9); self.w_sp.setSingleStep(0.5)
        self.w_sp.setKeyboardTracking(False)
        self.w_sp.setDecimals(1); self.w_sp.setFont(MONO); self.w_sp.setFixedWidth(80)
        self.w_act = lbl('—', 52, mono=True)
        self.w_err = lbl('—', 46, mono=True)
        self.w_t = lbl('—', 28, mono=True)
        self.w_tor = lbl('—', 46, mono=True)
        self.w_v = lbl('—', 36, mono=True)
        self.w_st = lbl('未标零', 150, align=Qt.AlignmentFlag.AlignLeft)
        self.w_home = btn('回零', self._home, 40)
        self.w_clr = btn('复位故障', lambda: ctrl.clear_error(self.mid), 72)
        ws = [self.w_id, self.w_name, self.w_on, self.w_en, self.w_zero, self.w_zv, self.w_sg, self.w_slider_panel, self.w_sp,
              self.w_act, self.w_err, self.w_t, self.w_tor, self.w_v, self.w_st, self.w_home, self.w_clr, self.w_verified]
        for c, w in enumerate(ws):
            grid.addWidget(w, r, c)
        self.w_sl.valueChanged.connect(lambda v: self._from_slider(v))
        self.w_sp.valueChanged.connect(lambda v: self._from_spin(v))

    def _from_slider(self, v):
        if self._sync:
            return
        if self.ctrl.set_target(self.mid, v / 2.0):
            self._set_pos(v / 2.0)
        else:
            self._set_pos(math.degrees(self.ctrl.states[self.mid].target))

    def _from_spin(self, v):
        if self._sync:
            return
        if self.ctrl.set_target(self.mid, v):
            self._set_pos(v)
        else:
            self._set_pos(math.degrees(self.ctrl.states[self.mid].target))

    def _update_range_label(self):
        self.w_range.setText(f'{self.w_sl.minimum() / 2:g}° 至 {self.w_sl.maximum() / 2:g}°')

    def _zoom_slider(self, factor):
        """独立缩放编辑视野，保留目标角；不能由范围变化触发运动。"""
        current = self.w_sl.value()
        span = max(20, abs(current), int(math.ceil(self.w_sl.maximum() * factor)))
        span = min(span, 2_000_000_000)  # Qt 整数滑条表示边界，不是关节限位。
        was_blocked = self.w_sl.blockSignals(True)
        try:
            self.w_sl.setRange(-span, span)
            self.w_sl.setValue(current)
        finally:
            self.w_sl.blockSignals(was_blocked)
        self._update_range_label()

    def _set_pos(self, deg):
        self._sync = True
        tick = int(round(deg * 2))
        if tick < self.w_sl.minimum() or tick > self.w_sl.maximum():
            span = max(360, abs(tick) + 20)
            self.w_sl.setRange(-span, span)
        self.w_sl.setValue(int(round(deg * 2))); self.w_sp.setValue(deg)
        self._update_range_label()
        self._sync = False

    def _toggle(self, checked):
        if checked:
            if not self.ctrl.set_enabled(self.mid, True):
                self.w_en.setChecked(False)
            else:
                self._set_pos(math.degrees(self.ctrl.states[self.mid].target))
        else:
            self.ctrl.set_enabled(self.mid, False)

    def _home(self):
        self._set_pos(0.0); self.ctrl.set_target(self.mid, 0.0)

    def _flip(self):
        try:
            self.w_sg.setText('+' if self.ctrl.flip_sign(self.mid) > 0 else '−')
        except OSError as e:
            QMessageBox.warning(self.w_sg, '保存方向失败', str(e))

    def refresh(self, snap, playing):
        fb = snap['fb']
        self.w_on.setStyleSheet(f'color:{GREEN if fb else GREY}')
        if playing and abs(self.w_sp.value() - snap['target']) > 0.05:
            self._set_pos(snap['target'])
        if snap['enabled'] != self.w_en.isChecked():
            self.w_en.setChecked(snap['enabled'])
        c = self.ctrl.calib.get(self.mid)
        self.w_zv.setText(f'{math.degrees(c["ex_zero"]):6.1f}' if c else '—')
        self.w_sg.setText('+' if self.ctrl.sign(self.mid) > 0 else '−')
        r = self.ctrl.ranges.get(self.mid)
        self.w_sl.setToolTip('默认 ±180°；扩大/缩小范围只改变编辑视野，不发运动指令。数值输入也可扩展；无关节角度限位。' +
                            (f'已采集范围 {r[0]:+.1f}..{r[1]:+.1f}°，仅记录。' if r else ''))
        warn = False
        if fb:
            center = self.ctrl.states[self.mid].center if snap['centered'] else 0.0
            act = math.degrees(fb['outputPos'] - center) if snap['centered'] else math.degrees(fb['outputPos'])
            self.w_act.setText(f'{act:+7.1f}')
            self.w_err.setText(f'{act - snap["cmd"]:+6.1f}' if snap['enabled'] else '—')
            self.w_t.setText(f'{fb["Temp"]:3d}')
            self.w_tor.setText(f'{fb["outputTor"]:+5.2f}')
            self.w_v.setText(f'{fb["vol"]:4.1f}')
            warn = (fb['Temp'] >= pc.TEMP_WARN or fb['vol'] < pc.VOLT_WARN or fb['vol'] > pc.VOLT_HIGH
                    or abs(fb['outputTor']) > pc.TORQUE_WARN)
            self.w_v.setStyleSheet(f'color:{ORANGE if fb["vol"] > pc.VOLT_HIGH else "#000"}')
        else:
            for w in (self.w_act, self.w_err, self.w_t, self.w_tor, self.w_v):
                w.setText('—')
        txt = snap['error'] or snap['status']
        self.w_st.setText(txt)
        self.w_st.setToolTip(txt)
        self.w_st.setStyleSheet(f'color:{RED if snap["error"] else (ORANGE if warn else "#000")}')




class Main(QMainWindow):
    def __init__(self, ctrl):
        super().__init__()
        self.setFont(QApplication.font())
        self.ctrl = ctrl
        self.lib = PoseLibrary(ctrl)
        self._worker = None
        self._probe_worker = None
        self._result = None
        self._closing = False
        self._allow_close = False
        self._measurement_dialog = None
        self._mapping_dialog = None
        self._workbench_dialog = None
        self._pose_path = pc.POSE_FILE
        from app_config import WINDOW_TITLE
        self.setWindowTitle(WINDOW_TITLE)
        outer = QScrollArea(); outer.setWidgetResizable(True); self.setCentralWidget(outer)
        root = QWidget(); outer.setWidget(root)
        v = QVBoxLayout(root); v.setContentsMargins(8, 6, 8, 6); v.setSpacing(4)

        connection = QHBoxLayout(); v.addLayout(connection)
        connection.addWidget(QLabel('串口'))
        self.port_combo = QComboBox(); self.port_combo.setMinimumWidth(240)
        self.port_combo.setSizeAdjustPolicy(QComboBox.SizeAdjustPolicy.AdjustToMinimumContentsLengthWithIcon)
        connection.addWidget(self.port_combo)
        self.b_refresh = btn('刷新串口', self._refresh_ports); connection.addWidget(self.b_refresh)
        connection.addWidget(QLabel(f'{ctrl.baud:,} bit/s'))
        self.b_connect = btn('连接', self._connection); connection.addWidget(self.b_connect)
        self.connection_status = QLabel('未连接'); connection.addWidget(self.connection_status, 1)
        stop = btn('急停卸力 (Esc)', self._estop); stop.setStyleSheet('color:#b00;font-weight:bold')
        connection.addWidget(stop)
        self._refresh_ports()

        # ---- 顶栏 ----
        self.control_panel = QWidget(); control_layout = QVBoxLayout(self.control_panel); control_layout.setContentsMargins(0, 0, 0, 0)
        top = QHBoxLayout(); control_layout.addLayout(top)
        v.addWidget(self.control_panel)
        top.addWidget(QLabel('速度 (°/s)'))
        self.speed = QSlider(Qt.Orientation.Horizontal); self.speed.setFixedWidth(180); self.speed.setRange(0, 10000)
        self.speed.setValue(self._speed_to_slider(pc.SPEED_DEFAULT)); self.speed.valueChanged.connect(self._speed)
        self.speed_spin = QDoubleSpinBox(); self.speed_spin.setDecimals(8); self.speed_spin.setRange(0, pc.SPEED_MAX); self.speed_spin.setValue(pc.SPEED_DEFAULT); self.speed_spin.setMinimumWidth(130); self.speed_spin.valueChanged.connect(self._speed_spin)
        self.speed_lbl = lbl(f'{pc.SPEED_DEFAULT:.1f}', 70, mono=True)
        self.speed.setToolTip(f'协议输出端速度范围 0–{pc.SPEED_MAX:.2f} °/s；默认值仅供实机安全起步，高速可能反灌母线')
        top.addWidget(self.speed); top.addWidget(self.speed_spin)
        top.addSpacing(8); top.addWidget(QLabel('刚度 Kp/Kd'))
        self.kp = QSlider(Qt.Orientation.Horizontal); self.kp.setFixedWidth(120); self.kp.setRange(0,10000); self.kp.setValue(self._gain_to_slider(ctrl.kp, pc.KP_MAX)); self.kp.valueChanged.connect(self._gain_sliders)
        self.kd = QSlider(Qt.Orientation.Horizontal); self.kd.setFixedWidth(100); self.kd.setRange(0,10000); self.kd.setValue(self._gain_to_slider(ctrl.kd, pc.KD_MAX)); self.kd.valueChanged.connect(self._gain_sliders)
        self.kp_spin = QDoubleSpinBox(); self.kp_spin.setDecimals(6); self.kp_spin.setRange(0,pc.KP_MAX); self.kp_spin.setValue(ctrl.kp); self.kp_spin.setMinimumWidth(118); self.kp_spin.valueChanged.connect(self._gain_spins)
        self.kd_spin = QDoubleSpinBox(); self.kd_spin.setDecimals(6); self.kd_spin.setRange(0,pc.KD_MAX); self.kd_spin.setValue(ctrl.kd); self.kd_spin.setMinimumWidth(105); self.kd_spin.valueChanged.connect(self._gain_spins)
        self.gain = QLabel('Kp 10 / Kd 0.5'); self.gain.setMinimumWidth(130); self.gain.setToolTip(f'协议输出端范围 Kp 0–{pc.KP_MAX:.3f} N·m/rad，Kd 0–{pc.KD_MAX:.4f} N·m·s/rad；极端值仅代表可编码')
        for w in (self.kp,self.kp_spin,self.kd,self.kd_spin,self.gain): top.addWidget(w)
        top.addStretch()
        top = QHBoxLayout(); control_layout.addLayout(top)
        for t, f in (('全部设零位', self._zero_all), ('全部使能', self._enable_all),
                     ('全部回零', self._home_all), ('全部卸力', self._release_all)):
            top.addWidget(btn(t, f, 72))
        self.b_range = QCheckBox('采集行程'); self.b_range.setToolTip('卸力后手摆采集角度范围，仅保存观察记录，不限制目标角')
        self.b_range.toggled.connect(self._range); top.addWidget(self.b_range)
        self.b_log = QCheckBox('记录遥测'); self.b_log.setToolTip('每秒记录位置、温度、电压、力矩到 log_时间.csv；位姿表另用“导出 CSV”')
        self.b_log.toggled.connect(self._log); top.addWidget(self.b_log)
        self.b_measure = QPushButton('实测采集'); self.b_measure.setToolTip('S288 纯位置、摆锤轨迹或总线时延原始采集；独占串口，停止时卸力')
        self.b_measure.clicked.connect(self._open_measurement); top.addWidget(self.b_measure)
        top.addStretch()
        persistence = QHBoxLayout(); v.addLayout(persistence)
        self.b_calib = btn('保存本机零位', self._save_calib); persistence.addWidget(self.b_calib)
        self.b_calib_export = btn('导出零位JSON', self._export_calib); persistence.addWidget(self.b_calib_export)
        self.b_mapping = btn('映射实测表', self._open_mapping); persistence.addWidget(self.b_mapping)
        self.b_workbench = btn('离线工作台', self._open_workbench); persistence.addWidget(self.b_workbench)
        self.b_workbench.setToolTip('构型草案、模型预检、平顺轨迹与参考导出；不发送电机命令')
        flash = btn('写入电机 Flash', lambda: None); flash.setEnabled(False)
        flash.setToolTip(pc.FLASH_ZERO_REASON); persistence.addWidget(flash)
        persistence.addWidget(QLabel('Flash：等待厂家原点协议'), 1)
        self.summary = lbl('', color='#666', align=Qt.AlignmentFlag.AlignLeft); persistence.addWidget(self.summary)
        calib_path = QLabel(f'本机零位：{pc.CALIB_FILE}'); calib_path.setWordWrap(True)
        calib_path.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse); v.addWidget(calib_path)
        QShortcut(QKeySequence('Escape'), self).activated.connect(self._estop)
        QShortcut(QKeySequence('Ctrl+Space'), self).activated.connect(self._stop_playback)

        # ---- 舵机表 ----
        joints = QWidget(); grid = QGridLayout(joints); grid.setHorizontalSpacing(6); grid.setVerticalSpacing(3)
        grid.setColumnStretch(7, 1)                     # 多出来的宽度全给滑条列，别挤到别的列
        joint_scroll = QScrollArea(); joint_scroll.setWidgetResizable(True); joint_scroll.setWidget(joints)
        joint_scroll.setMinimumHeight(240); v.addWidget(joint_scroll, 3)
        for c, h in enumerate(Row.HEADS):
            grid.addWidget(lbl(h, color='#666', align=Qt.AlignmentFlag.AlignLeft), 0, c)
        self.rows = {}
        r = 1
        for g in pc.GROUP_ORDER:
            ids = [m for m in ctrl.states if ctrl.states[m].spec['group'] == g]
            if not ids:
                continue
            grid.addWidget(hline(), r, 0, 1, len(Row.HEADS)); r += 1
            for mid in ids:
                self.rows[mid] = Row(grid, r, ctrl, ctrl.states[mid].spec, self._zero_one); r += 1

        # ---- 位姿面板 ----
        pf = QGroupBox('位姿编排'); v.addWidget(pf, 2)
        pv = QVBoxLayout(pf); pv.setSpacing(3)
        bar = QHBoxLayout(); pv.addLayout(bar)
        bar.addWidget(QLabel('名称'))
        self.pname = QLineEdit('pose1'); self.pname.setFixedWidth(120); bar.addWidget(self.pname)
        self.r_cmd = QRadioButton('指令角'); self.r_act = QRadioButton('实测角（手动示教）')
        self.r_cmd.setChecked(True); bar.addWidget(self.r_cmd); bar.addWidget(self.r_act)
        for t, f, w in (('记录位姿', self._record, 68), ('覆盖选中', self._overwrite, 68),
                        ('删除', self._delete, 44), ('上移', lambda: self._move(-1), 40), ('下移', lambda: self._move(1), 40)):
            bar.addWidget(btn(t, f, w))
        bar.addStretch()
        files = QHBoxLayout(); pv.addLayout(files)
        for t, f in (('打开 JSON', self._load_dlg), ('保存', self._save), ('导出 JSON', self._save_as), ('导出 CSV', self._export_csv)):
            files.addWidget(btn(t, f))
        self.pose_path_label = QLabel(self._pose_path); self.pose_path_label.setWordWrap(True)
        files.addWidget(self.pose_path_label, 1)

        bar2 = QHBoxLayout(); pv.addLayout(bar2)
        bar2.addWidget(lbl('新位姿默认', color='#666')); bar2.addSpacing(6)
        bar2.addWidget(QLabel('运动速度 (°/s)'))
        self.pose_speed = QDoubleSpinBox(); self.pose_speed.setDecimals(8); self.pose_speed.setRange(.01, pc.SPEED_MAX); self.pose_speed.setSingleStep(0.5)
        self.pose_speed.setValue(pc.SPEED_DEFAULT); self.pose_speed.setMinimumWidth(130); self.pose_speed.setToolTip('新动作默认指令速度；已记录动作双击表格修改。播放中顶栏手动速度不生效'); bar2.addWidget(self.pose_speed)
        bar2.addWidget(QLabel('动作结束暂停 (s)'))
        self.dwell = QDoubleSpinBox(); self.dwell.setRange(0.0, 10.0); self.dwell.setSingleStep(0.5)
        self.dwell.setDecimals(1); self.dwell.setValue(1.0); self.dwell.setFixedWidth(84); bar2.addWidget(self.dwell)
        bar2.addSpacing(8); bar2.addWidget(lbl('播放', color='#666')); bar2.addSpacing(4)
        bar2.addWidget(QLabel('遍数'))
        self.repeat = QSpinBox(); self.repeat.setRange(0, 999); self.repeat.setValue(1); self.repeat.setFixedWidth(72)
        self.repeat.setSpecialValueText('∞'); self.repeat.setToolTip('0 = 无限循环，按停止结束')
        bar2.addWidget(self.repeat)
        self.b_play = btn('按顺序播放', self._play, 84); bar2.addWidget(self.b_play)
        self.b_go2 = btn('执行选中动作', self._goto_smooth, 110); bar2.addWidget(self.b_go2)
        self.b_stop = btn('停止播放', self._stop_playback, 68); bar2.addWidget(self.b_stop)
        bar2.addStretch()
        bar3 = QHBoxLayout(); pv.addLayout(bar3)
        bar3.addWidget(QLabel('选中帧与下一帧之间插入'))
        self.nmid = QSpinBox(); self.nmid.setRange(1, 20); self.nmid.setValue(3); self.nmid.setFixedWidth(64)
        bar3.addWidget(self.nmid); bar3.addWidget(QLabel('个过渡帧'))
        bar3.addWidget(btn('生成', self._insert_mid, 44)); bar3.addStretch()

        self.cols = list(ctrl.states)
        self.NFIX = 3                                   # 位姿名、速度、动作结束暂停，后面才是各 ID
        self.tree = QTableWidget(0, self.NFIX + len(self.cols))
        self.tree.setHorizontalHeaderLabels(['位姿名称', '运动速度 (°/s)', '动作结束暂停 (s)'] +
                                           [f'ID {m}\n{ctrl.states[m].spec["name"]} (°)' for m in self.cols])
        for c, m in enumerate(self.cols, self.NFIX):
            self.tree.horizontalHeaderItem(c).setToolTip(f'ID {m} · {ctrl.states[m].spec["name"]} · ' +
                                                        ('已确认' if ctrl.states[m].spec['verified'] else '映射待实测确认'))
        self.tree.verticalHeader().setVisible(False)
        self.tree.setSelectionBehavior(QAbstractItemView.SelectionBehavior.SelectRows)
        self.tree.setSelectionMode(QAbstractItemView.SelectionMode.SingleSelection)
        self.tree.setEditTriggers(QAbstractItemView.EditTrigger.DoubleClicked | QAbstractItemView.EditTrigger.EditKeyPressed)
        self.tree.setToolTip('双击修改位姿名称、运动速度、动作结束暂停或角度；回车生效。横向滚动查看其他关节。')
        hh = self.tree.horizontalHeader(); hh.setSectionResizeMode(QHeaderView.ResizeMode.Interactive)
        hh.setDefaultSectionSize(125)
        self.tree.setColumnWidth(0, 170)
        self.tree.itemChanged.connect(self._cell_edited)
        self._filling = False
        self.tree.verticalHeader().setDefaultSectionSize(28)
        self.tree.setMinimumHeight(120)
        pv.addWidget(self.tree, 1)
        self.msg = lbl('选择串口并连接 → 核对关节 → 设零位 → 示教记录 → 导出位姿。',
                       color='#666', align=Qt.AlignmentFlag.AlignLeft)
        pv.addWidget(self.msg)
        self.msg.setWordWrap(True)

        if os.path.exists(pc.POSE_FILE):
            self._load(pc.POSE_FILE)
        self.t1 = QTimer(self); self.t1.timeout.connect(self._tick); self.t1.start(100)
        self.t2 = QTimer(self); self.t2.timeout.connect(self._probe_tick); self.t2.start(1000)
        self._probe_tick()
        self._tick()
        self.resize(1280, 850)

    # ---- 顶栏 ----
    def _busy(self):
        return self._worker is not None

    def _refresh_ports(self):
        selected = self.port_combo.currentData() or self.ctrl.port
        self.port_combo.clear()
        if self.ctrl.sim:
            self.port_combo.addItem('离线仿真（不连接硬件）', 'SIM')
            return
        try:
            for device, description in pc.port_details():
                self.port_combo.addItem(f'{device} · {description}', device)
        except Exception as e:
            self.connection_status.setText(f'枚举串口失败：{e}')
        idx = self.port_combo.findData(selected)
        if idx >= 0:
            self.port_combo.setCurrentIndex(idx)
        elif selected and self.ctrl.port == selected:
            self.port_combo.addItem(f'{selected} · 指定串口（未枚举到）', selected)
            self.port_combo.setCurrentIndex(self.port_combo.count() - 1)
        if not self.port_combo.count():
            self.port_combo.addItem('未发现串口，请插入设备后刷新', None)

    def _task(self, label, fn):
        """后台只读写核心状态，结果由 GUI 定时器领取。维护操作串行执行。"""
        if self._busy():
            return False
        self.say(label, BLUE)
        self._result = None
        def run():
            try:
                if self._probe_worker:
                    self._probe_worker.join()
                result = fn()
                self._result = (str(result or label + '完成'), '#060')
            except Exception as e:
                self._result = (f'{label}失败：{e}', RED)
        self._worker = threading.Thread(target=run, daemon=True)
        self._worker.start()
        self._tick()
        return True

    def _connection(self):
        if self._busy():
            return
        if self.ctrl.connected:
            self.lib.stop.set()
            self.ctrl.release_all()
            self.b_log.setChecked(False)
            self._task('正在断开', self._disconnect)
        else:
            port = self.port_combo.currentData()
            if not port:
                self.say('未发现串口，请插入设备后刷新', RED)
                return
            self.ctrl.port = port
            def connect():
                if not self.ctrl.connect():
                    raise RuntimeError(self.ctrl.conn_msg)
                return self.ctrl.last_error or f'已连接：{self.ctrl.conn_msg}；电机在线状态见关节表'
            self._task('正在连接', connect)

    def _disconnect(self):
        self.lib.halt()
        if self.ctrl.range_rec:
            try:
                self.ctrl.stop_range()
            except Exception as e:
                self.ctrl.last_error = f'行程保存失败：{e}'
        self.ctrl.shutdown()
        return '已断开，位姿库保留'

    def _save_calib(self):
        self._task('保存本机零位', lambda: (self.ctrl.save_calib(), f'已保存本机零位：{pc.CALIB_FILE}')[1])

    def _zero_one(self, mid):
        if self.lib.playing() or self.ctrl.range_rec:
            self.say('请先停止播放及行程采集', RED)
            return
        self._task(f'标定 ID {mid}', lambda: f'ID {mid}：' +
                   ('零位已保存到本机' if self.ctrl.zero_now(mid) else self.ctrl.states[mid].status))

    @staticmethod
    def _speed_to_slider(v): return round(math.sqrt(max(0.0, min(v, pc.SPEED_MAX)) / pc.SPEED_MAX) * 10000)
    @staticmethod
    def _gain_to_slider(v, maximum): return round(math.sqrt(max(0.0, min(v, maximum)) / maximum) * 10000)
    def _speed(self, v):
        value = (v / 10000) ** 2 * pc.SPEED_MAX
        self.speed_spin.blockSignals(True); self.speed_spin.setValue(value); self.speed_spin.blockSignals(False)
        self.speed_lbl.setText(f'{value:.2f}'); self.ctrl.set_speed(value)
    def _speed_spin(self, value):
        value = min(value, pc.SPEED_MAX)
        self.speed.blockSignals(True); self.speed.setValue(self._speed_to_slider(value)); self.speed.blockSignals(False)
        self.speed_lbl.setText(f'{value:.2f}'); self.ctrl.set_speed(value)
    def _gain_sliders(self):
        kp=(self.kp.value()/10000)**2*pc.KP_MAX; kd=(self.kd.value()/10000)**2*pc.KD_MAX
        self.kp_spin.blockSignals(True); self.kp_spin.setValue(kp); self.kp_spin.blockSignals(False)
        self.kd_spin.blockSignals(True); self.kd_spin.setValue(kd); self.kd_spin.blockSignals(False); self.ctrl.set_gain_values(kp,kd); self.gain.setText(f'Kp {kp:.3g} / Kd {kd:.3g}')
    def _gain_spins(self):
        kp=self.kp_spin.value(); kd=self.kd_spin.value(); self.kp.blockSignals(True); self.kp.setValue(self._gain_to_slider(kp,pc.KP_MAX)); self.kp.blockSignals(False); self.kd.blockSignals(True); self.kd.setValue(self._gain_to_slider(kd,pc.KD_MAX)); self.kd.blockSignals(False); self.ctrl.set_gain_values(kp,kd); self.gain.setText(f'Kp {kp:.3g} / Kd {kd:.3g}')

    def say(self, text, color='#060'):
        self.msg.setText(text); self.msg.setStyleSheet(f'color:{color}')

    def _zero_all(self):
        if self.lib.playing() or self.ctrl.range_rec:
            self.say('请先停止播放及行程采集', RED); return
        if any(s.enabled for s in self.ctrl.states.values()):
            self.say('请先支撑机器人并全部卸力，再标定零位', RED); return
        if QMessageBox.question(self, '全部标零', '把所有在线舵机的当前位置记为零位并写入文件？\n请确认机器人已摆到标准站姿。') \
                != QMessageBox.StandardButton.Yes:
            return
        def calibrate():
            mids = [m for m, st in self.ctrl.states.items() if st.fb]
            n = sum(1 for m in mids if self.ctrl.zero_now(m))
            return f'本机零位已保存 {n}/{len(mids)} 台；失败原因见关节状态'
        self._task('正在标定本机零位', calibrate)

    def _enable_all(self):
        for m, st in self.ctrl.states.items():
            if st.fb and st.centered and not st.error:
                self.ctrl.set_enabled(m, True)

    def _home_all(self):
        for r in self.rows.values():
            r._home()

    def _release_all(self):
        self.lib.stop.set()
        self.ctrl.release_all()

    def _range(self, on):
        if on:
            if self.lib.playing() or not self.ctrl.start_range():
                self.b_range.blockSignals(True); self.b_range.setChecked(False); self.b_range.blockSignals(False)
                self.say('需至少一台在线、已标定且已卸力的关节，且播放已停止', RED); return
            self.say('录行程中：卸力的台用手慢慢摆到两端，摆完取消勾选', BLUE)
        else:
            self._task('保存行程', lambda: f'已保存 {self.ctrl.stop_range()} 台行程记录，不作为角度限位')

    def _log(self, on):
        if on:
            import csv
            from datetime import datetime
            path = os.path.join(pc.DATA_DIR, f'log_{datetime.now():%Y%m%d_%H%M%S}.csv')
            try:
                self._logf = open(path, 'w', newline='', encoding='utf-8-sig')
            except OSError as e:
                self.b_log.blockSignals(True); self.b_log.setChecked(False); self.b_log.blockSignals(False)
                self.say(f'无法创建遥测文件：{e}', RED); return
            self._logw = csv.writer(self._logf)
            self._logw.writerow(['time', 'id', 'enabled', 'cmd_deg', 'act_deg', 'temp', 'vol', 'tor'])
            self.say(f'记录到 {os.path.basename(path)}')
        else:
            f = getattr(self, '_logf', None)
            if f:
                f.close(); self._logf = None
            self.say('CSV 已停')

    def _open_measurement(self):
        if self._busy() or self.lib.playing() or not self.ctrl.connected or self.ctrl.sim or self.ctrl.range_rec:
            self.say('请先连接真实串口并停止播放，再开始实测采集', RED); return
        if self._measurement_dialog is None:
            self._measurement_dialog = MeasurementDialog(self, self.ctrl)
        self._measurement_dialog.show(); self._measurement_dialog.raise_(); self._measurement_dialog.activateWindow()

    def _open_workbench(self):
        """只读快照进入独立工作台，不传控制器、不认证旧方向或站姿。"""
        if self._closing:
            return
        if self._workbench_dialog is not None:
            self._workbench_dialog.show()
            self._workbench_dialog.raise_()
            self._workbench_dialog.activateWindow()
            return
        with self.ctrl.lock:
            profile = draft_from_hardware([s.spec for s in self.ctrl.states.values()])
            # 现有反馈没有接收时间戳，因此只标为快照，绝不标为实时起点。
            start = {mid: math.degrees(s.fb['outputPos'] - s.center)
                     for mid, s in self.ctrl.states.items()
                     if self.ctrl.connected and s.centered and s.fb and not s.error
                     and s.comm_fail == 0 and math.isfinite(s.fb['outputPos'] - s.center)}
        dialog = WorkbenchDialog(self, profile, self.lib.poses, start or None)
        self._workbench_dialog = dialog
        def finished():
            if self._workbench_dialog is dialog:
                self._workbench_dialog = None
            dialog.deleteLater()
        dialog.finished.connect(finished)
        dialog.show()

    def _export_calib(self):
        if self._busy():return
        path,_=QFileDialog.getSaveFileName(self,'导出电脑端零位','calib_host_xianduck.json','JSON (*.json)')
        if path:
            try:self.ctrl.export_calibration(path); self.say(f'已导出零位：{path}')
            except Exception as exc:self.say(str(exc),RED)

    def _open_mapping(self):
        if self._busy():return
        if self._mapping_dialog is None:self._mapping_dialog=MappingDialog(self,self.ctrl)
        self._mapping_dialog.show(); self._mapping_dialog.raise_()

    def _estop(self):
        self.lib.stop.set()
        self.ctrl.release_all()
        self.say('急停：已全部卸力，播放已停', RED)

    def _stop_playback(self):
        # 先请求停止；下一次 GUI tick 在线程退出后冻结尚未完成的目标。
        self.lib.stop.set()
        self._freeze_pending = True
        self.say('停止播放，保持当前指令位置；卸力请按急停', BLUE)

    def _can_edit(self):
        if self.lib.playing() or self._busy():
            self.say('请先停止播放并等待当前操作结束', ORANGE)
            return False
        return True

    # ---- 位姿 ----
    def _sel(self):
        rows = self.tree.selectionModel().selectedRows()
        return rows[0].row() if rows else None

    def _refresh_tree(self, sel=None):
        self._filling = True
        self.tree.setRowCount(len(self.lib.poses))
        for i, p in enumerate(self.lib.poses):
            self.tree.setItem(i, 0, QTableWidgetItem(p['name']))
            self.tree.setItem(i, 1, QTableWidgetItem(f'{p.get("speed", self.ctrl.speed):.2f}'))
            self.tree.setItem(i, 2, QTableWidgetItem(f'{p.get("dwell", 1.0):.1f}'))
            for c, m in enumerate(self.cols, self.NFIX):
                it = QTableWidgetItem(f'{p["angles"][m]:+.1f}' if m in p['angles'] else '·')
                it.setTextAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
                self.tree.setItem(i, c, it)
        self._filling = False
        if sel is not None and self.lib.poses:
            self.tree.selectRow(min(sel, len(self.lib.poses) - 1))

    def _cell_edited(self, it):
        """表格里直接改：名字、运动速度、动作结束暂停、单个 ID 的角度。"""
        if self._filling:
            return
        if not self._can_edit():
            self._refresh_tree(); return
        i, c, txt = it.row(), it.column(), it.text().strip()
        if c == 0:
            self.lib.poses[i]['name'] = txt or self.lib.poses[i]['name']
        else:
            try:
                v = float(txt)
                if not math.isfinite(v):
                    raise ValueError('角度或时长必须为有限数值')
            except ValueError:
                self._refresh_tree(i); return
            if c == 1:
                try: self.lib.set_timing(i, speed=v)
                except ValueError as e: self.say(str(e), RED)
            elif c == 2:
                try: self.lib.set_timing(i, dwell=v)
                except ValueError as e: self.say(str(e), RED)
            else:
                try:
                    self.lib.set_angle(i, self.cols[c - self.NFIX], v)
                except ValueError as e:
                    self.say(str(e), RED)
        self._refresh_tree(i)

    def _record(self):
        if not self._can_edit():
            return
        if self.ctrl.unzeroed():
            self.say('请先标定所有在线关节，避免将原始角误记为相对零位角', RED); return
        p = self.lib.record(self.pname.text(), self.r_act.isChecked(), self.pose_speed.value(), self.dwell.value())
        if p is None:
            self.say('无在线电机，无法记录位姿', RED); return
        self.pname.setText(f'pose{len(self.lib.poses) + 1}')
        self._refresh_tree(len(self.lib.poses) - 1)
        uz = self.ctrl.unzeroed()
        warn = f'；未标零 {uz}，这些台记的是原始角' if uz else ''
        self.say(f'已记录 {p["name"]}（{len(p["angles"])} 台）{warn}', ORANGE if uz else '#060')

    def _overwrite(self):
        if not self._can_edit():
            return
        if not self.ctrl.read_angles(self.r_act.isChecked()):
            self.say('无在线电机，未覆盖原位姿', RED); return
        if self.ctrl.unzeroed():
            self.say('在线关节尚未全部标定，未覆盖原位姿', RED); return
        i = self._sel()
        if i is not None:
            self.lib.overwrite(i, self.r_act.isChecked()); self._refresh_tree(i)

    def _goto_sel(self):
        if not self._can_edit() or not self.ctrl.connected or self.ctrl.range_rec:
            return
        i = self._sel()
        if i is not None:
            self.ctrl.motion_blocked = False
            try:
                self.lib.goto_smooth(i)
                self.say(f'执行 {self.lib.poses[i]["name"]}', BLUE)
            except ValueError as e:
                self.say(str(e), RED)

    def _goto_smooth(self):
        if not self._can_edit() or not self.ctrl.connected or self.ctrl.range_rec:
            return
        i = self._sel()
        if i is not None:
            self.ctrl.motion_blocked = False
            self.lib.goto_smooth(i)

    def _delete(self):
        if not self._can_edit():
            return
        i = self._sel()
        if i is not None:
            self.lib.delete(i); self._refresh_tree(i)

    def _move(self, d):
        if not self._can_edit():
            return
        i = self._sel()
        if i is not None:
            self._refresh_tree(self.lib.move(i, d))

    def _insert_mid(self):
        if not self._can_edit():
            return
        i = self._sel()
        if i is None or i + 1 >= len(self.lib.poses):
            self.say('先选中一条，且它下面还要有一条', RED); return
        a, b = self.lib.poses[i]['name'], self.lib.poses[i + 1]['name']
        n = self.lib.insert_mid(i, self.nmid.value())
        self._refresh_tree(i)
        self.say(f'在 {a} 和 {b} 之间插入 {n} 个中间位姿')

    def _play(self):
        if not self.ctrl.connected or self._busy() or self.ctrl.range_rec:
            self.say('请连接设备并停止维护操作及行程采集', RED); return
        self.ctrl.motion_blocked = False
        if not self.lib.play(self.repeat.value()):
            self.say('列表为空或正在播放', RED)

    def _save(self, path=None):
        path = path or self._pose_path
        try:
            self.lib.save(path)
        except (OSError, ValueError) as e:
            self.say(f'保存失败：{e}', RED); return
        self.say(f'已保存 {len(self.lib.poses)} 个位姿 → {path}')

    def _save_as(self):
        p, _ = QFileDialog.getSaveFileName(self, '导出位姿 JSON', self._pose_path, 'JSON (*.json)')
        if p:
            self._save(p)

    def _export_csv(self):
        p, _ = QFileDialog.getSaveFileName(self, '导出位姿 CSV', os.path.join(pc.DATA_DIR, 'poses_xianduck.csv'), 'CSV (*.csv)')
        if not p:
            return
        try:
            self.lib.export_csv(p)
            self.say(f'已导出 {len(self.lib.poses)} 帧位姿 → {p}')
        except OSError as e:
            self.say(f'导出失败：{e}', RED)

    def _load_dlg(self):
        if not self._can_edit():
            return
        p, _ = QFileDialog.getOpenFileName(self, '加载位姿', pc.DATA_DIR, 'JSON (*.json)')
        if p:
            self._load(p)

    def _load(self, path):
        try:
            self.lib.load(path)
        except Exception as e:              # noqa: BLE001
            self.say(f'读位姿文件失败: {e}', RED); return
        self._refresh_tree()
        self._pose_path = path
        self.pose_path_label.setText(path)
        self.say(f'已加载 {len(self.lib.poses)} 个位姿 ← {os.path.basename(path)}；{self.lib.load_warning}')

    # ---- 周期 ----
    def _tick(self):
        if self._worker and not self._worker.is_alive():
            self._worker = None
            if self._result:
                self.say(*self._result)
                self._result = None
        if self._closing and not self._busy():
            if self.ctrl.connected:
                self._task('正在安全退出', self._disconnect)
            elif self._workbench_dialog is not None:
                self._workbench_dialog.reject()
            else:
                self._allow_close = True
                self.close()
            return
        busy = self._busy()
        connected = self.ctrl.connected
        self.connection_status.setText(('正在处理连接/维护任务…' if busy else self.ctrl.conn_msg))
        self.connection_status.setToolTip(self.ctrl.conn_msg)
        self.b_connect.setText('断开' if connected else '连接')
        self.b_connect.setEnabled(not busy)
        self.b_refresh.setEnabled(not busy and not connected and not self.ctrl.sim)
        self.port_combo.setEnabled(not busy and not connected and not self.ctrl.sim)
        self.control_panel.setEnabled(connected and not busy and not getattr(self.ctrl, 'measurement_active', False))
        self.b_calib.setEnabled(not busy and bool(self.ctrl.calib))
        self.b_calib_export.setEnabled(not busy and bool(self.ctrl.calib))
        self.b_mapping.setEnabled(not busy)
        self.b_workbench.setEnabled(not self._closing)
        if not connected:
            self.b_range.blockSignals(True); self.b_range.setChecked(False); self.b_range.blockSignals(False)
        snap = self.ctrl.snapshot()
        playing = self.lib.playing()
        self.speed.setEnabled(not playing); self.speed_spin.setEnabled(not playing)
        self.gain.setEnabled(not playing and not self.ctrl.range_rec)
        for w in (self.kp,self.kd,self.kp_spin,self.kd_spin):
            w.setEnabled(not playing and not self.ctrl.range_rec)
        self.b_range.setEnabled(not playing)
        if getattr(self, '_freeze_pending', False) and not playing:
            with self.ctrl.lock:
                for st in self.ctrl.states.values():
                    st.target = st.cmd
            self._freeze_pending = False
        for mid, r in self.rows.items():
            r.refresh(snap[mid], playing)
            for w in (r.w_en, r.w_zero, r.w_sg, r.w_sl, r.w_sp, r.w_home, r.w_clr):
                w.setEnabled(connected and not busy and not playing and not self.ctrl.range_rec)
        n = len(snap)
        on = sum(1 for s in snap.values() if s['fb'])
        zc = sum(1 for s in snap.values() if s['centered'])
        en = sum(1 for s in snap.values() if s['enabled'])
        er = sum(1 for s in snap.values() if s['error'])
        self.summary.setText(f'在线 {on}/{n}  已标定 {zc}/{n}  使能 {en}/{n}' + (f'  故障 {er}' if er else ''))
        self.summary.setStyleSheet(f'color:{RED if er else ("#060" if on == n else ORANGE)}')
        if self.lib.msg:
            self.say(self.lib.msg, BLUE); self.lib.msg = ''
        self.b_play.setEnabled(connected and not busy and not playing)
        self.b_go2.setEnabled(connected and not busy and not playing); self.b_stop.setEnabled(playing)
        self.b_measure.setEnabled(connected and not self.ctrl.sim and not busy and not playing and not getattr(self.ctrl, 'measurement_active', False))

    def _probe_tick(self):
        if self._closing or self._busy() or getattr(self.ctrl, 'measurement_active', False) or not self.ctrl.connected:
            return
        if not self.ctrl.range_rec and not (self._probe_worker and self._probe_worker.is_alive()):
            self._probe_worker = threading.Thread(target=self.ctrl.refresh_offline, daemon=True)
            self._probe_worker.start()
        f = getattr(self, '_logf', None)
        if f:
            import time as _t
            t = f'{_t.time():.1f}'
            snap = self.ctrl.snapshot()
            for mid, s in snap.items():
                fb = s['fb']
                if not fb:
                    continue
                c = self.ctrl.states[mid].center if s['centered'] else 0.0
                self._logw.writerow([t, mid, int(s['enabled']), f'{s["cmd"]:.2f}',
                                     f'{math.degrees(fb["outputPos"] - c):.2f}',
                                     fb['Temp'], fb['vol'], f'{fb["outputTor"]:.3f}'])
            f.flush()

    def closeEvent(self, ev):
        if self._allow_close:
            ev.accept(); return
        ev.ignore()
        self._closing = True
        if self._workbench_dialog is not None:
            self._workbench_dialog.reject()
        self.lib.stop.set()
        self.ctrl.release_all()
        f = getattr(self, '_logf', None)
        if f:
            f.close()
            self._logf = None
        self._tick()


def main():
    ap = argparse.ArgumentParser(description='标零与位姿记录上位机')
    ap.add_argument('--port', default=None, help='指定串口并连接，例如 COM5；默认在主窗口选择后连接')
    ap.add_argument('--baud', type=int, default=6000000)
    ap.add_argument('--sim', action='store_true')
    ap.add_argument('--title', help='覆盖本次运行的窗口标题；默认读取构建时 app_config.py')
    ap.add_argument('--ids', help='逗号分隔，默认 1-5,9-13,6,7,8,14')
    ap.add_argument('--list-ports', action='store_true', help='列出串口后退出')
    ap.add_argument('--no-watchdog', action='store_true', help='关闭舵机超时位看门狗（默认开：1 s 无指令舵机自停）')
    a = ap.parse_args()
    if a.list_ports:
        print('\n'.join(list_ports()) or '(没有串口)'); return 0
    ids = [int(x) for x in a.ids.split(',')] if a.ids else pc.DEFAULT_IDS
    if not ids or len(ids) != len(set(ids)) or any(m < 0 or m > 14 for m in ids):
        ap.error('ID 必须为 0–14 且不能重复；15 为广播地址')
    sim = a.sim
    app = QApplication(sys.argv)
    configure_font(app)
    if sys.platform == 'win32' and not sim:
        import ctypes
        ctypes.windll.kernel32.CreateMutexW.restype = ctypes.c_void_p
        mutex = ctypes.windll.kernel32.CreateMutexW(None, False, 'xianduck_pose_ui_single_instance')
        if ctypes.windll.kernel32.GetLastError() == 183:          # ERROR_ALREADY_EXISTS
            QMessageBox.critical(None, '已经开着一个', '上位机已经在运行，总线只能有一个主机。先关掉那个窗口。')
            return 3
    ctrl = PoseController(a.port, a.baud, sim, ids, watchdog=not a.no_watchdog)
    win = Main(ctrl)
    if a.title is not None:
        win.setWindowTitle(a.title)
    win.show()
    if a.port or sim:
        QTimer.singleShot(0, win._connection)
    rc = app.exec()
    ctrl.shutdown()
    if sys.platform == 'win32' and not sim and mutex:
        ctypes.windll.kernel32.CloseHandle(ctypes.c_void_p(mutex))
    return rc


if __name__ == '__main__':
    sys.exit(main())
