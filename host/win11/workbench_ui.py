"""离线构型与轨迹工作台。只接收数据副本，不持有电机控制器。"""
from bisect import bisect_left
from copy import deepcopy
import math
import threading

from PyQt6.QtCore import Qt, QTimer, QPointF
from PyQt6.QtGui import QColor, QPainter, QPainterPath, QPen
from PyQt6.QtWidgets import (QDialog, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton,
                             QTabWidget, QLineEdit, QTableWidget, QTableWidgetItem, QHeaderView,
                             QFileDialog, QMessageBox, QComboBox, QDoubleSpinBox, QSlider,
                             QPlainTextEdit, QGroupBox)

import robot_profile as rp
from model_preflight import inspect_model
from trajectory import plan_poses
from trajectory_export import export_reference

FIELDS = ('motor_id', 'semantic', 'model_joint', 'bus', 'direction', 'q_home_rad',
          'role', 'policy_index', 'verified')
HEADERS = ('ID', '关节语义', '模型关节名', '总线', '方向 ±1', '零位对应模型角 rad',
           '角色', '策略序号', '映射确认')


class CurveView(QWidget):
    """单关节三导数曲线，时间游标可拖动；无需安装绘图库。"""
    def __init__(self):
        super().__init__()
        self.setMinimumSize(600, 250)
        self.plan = None
        self.joint = 0
        self.cursor = 0.0
        self.setAccessibleName('关节位置速度加速度曲线')

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.fillRect(self.rect(), QColor('#fafafa'))
        if not self.plan:
            painter.drawText(self.rect(), Qt.AlignmentFlag.AlignCenter, '生成轨迹后查看位置、速度和加速度')
            return
        samples = self.plan['samples']
        duration = max(self.plan['duration_s'], 1e-9)
        stride = max(1, len(samples)//2000)
        drawn = samples[::stride]
        if drawn[-1] is not samples[-1]:
            drawn = drawn + [samples[-1]]
        for band, (key, label, color) in enumerate((('q_deg', '位置 °', '#1664ad'),
                ('dq_deg_s', '速度 °/s', '#167d4b'), ('ddq_deg_s2', '加速度 °/s²', '#a44b1b'))):
            top = 24 + band*(self.height()-35)/3
            height = (self.height()-65)/3
            left, width = 90, max(1, self.width()-108)
            values = [s[key][self.joint] for s in samples]
            low, high = min(values), max(values)
            spread = max(high-low, 1e-6)
            painter.setPen(QColor('#666'))
            painter.drawText(4, int(top+14), label)
            painter.drawText(4, int(top+32), f'{low:.3g} … {high:.3g}')
            painter.drawLine(left, int(top+height), left+width, int(top+height))
            path = QPainterPath()
            for i, sample in enumerate(drawn):
                point = QPointF(left+width*sample['t_s']/duration,
                                top+height-height*(sample[key][self.joint]-low)/spread)
                path.moveTo(point) if i == 0 else path.lineTo(point)
            painter.setPen(QPen(QColor(color), 1.5)); painter.drawPath(path)
            painter.setPen(QPen(QColor('#bb3333'), 1))
            x = int(left+width*self.cursor/duration)
            painter.drawLine(x, int(top), x, int(top+height))
        painter.setPen(QColor('#555'))
        painter.drawText(90, self.height()-3, f'0 s                         计划时长 {self.plan["duration_s"]:.3f} s')


class WorkbenchDialog(QDialog):
    def __init__(self, parent, profile, poses, start_angles=None):
        super().__init__(parent)
        self.setWindowTitle('构型与平顺轨迹工作台 · 离线')
        self.resize(1180, 820)
        self.profile = deepcopy(profile)
        self.baseline = deepcopy(profile)
        self.poses = deepcopy(poses)
        self.start_angles = deepcopy(start_angles)
        self.plan = None
        self.report = None
        self._thread = None
        self._result = None
        self._cancel = threading.Event()
        self._closing = False
        self._close_result = QDialog.DialogCode.Rejected
        self._filling = False
        layout = QVBoxLayout(self)
        banner = QLabel('离线设计 · 不发送电机命令。构型更改不影响当前实机；平顺预览不等于物理仿真或安全认证。')
        banner.setWordWrap(True); layout.addWidget(banner)
        self.tabs = QTabWidget(); layout.addWidget(self.tabs)
        self._profile_tab(); self._motion_tab()
        bottom = QHBoxLayout(); layout.addLayout(bottom)
        self.status = QLabel('可打开已有位姿 JSON，或使用明确标注的离线示例。'); self.status.setWordWrap(True)
        bottom.addWidget(self.status, 1)
        self.cancel_button = QPushButton('取消任务'); self.cancel_button.clicked.connect(self._cancel.set)
        self.cancel_button.setEnabled(False); bottom.addWidget(self.cancel_button)
        self._fill_profile(profile)
        self._fill_poses()
        self.timer = QTimer(self); self.timer.timeout.connect(self._poll); self.timer.start(50)

    def _button(self, layout, text, callback):
        button = QPushButton(text); button.clicked.connect(callback); layout.addWidget(button)
        return button

    def _profile_tab(self):
        page = QWidget(); box = QVBoxLayout(page); self.tabs.addTab(page, '构型与模型预检')
        bar = QHBoxLayout(); box.addLayout(bar)
        bar.addWidget(QLabel('构型名称')); self.profile_name = QLineEdit(); bar.addWidget(self.profile_name, 1)
        self.profile_name.textChanged.connect(self._profile_changed)
        self._button(bar, '打开构型', self._load_profile)
        self._button(bar, '保存草案', self._save_profile)
        self._button(bar, '检查映射与差异', self._check_profile)
        self.table = QTableWidget(0, len(FIELDS)); self.table.setHorizontalHeaderLabels(HEADERS)
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Interactive)
        self.table.horizontalHeader().setDefaultSectionSize(95)
        self.table.setColumnWidth(1, 145); self.table.setColumnWidth(2, 225)
        self.table.setColumnWidth(5, 150)
        self.table.itemChanged.connect(self._profile_changed); box.addWidget(self.table, 2)
        row = QHBoxLayout(); box.addLayout(row)
        self._button(row, '添加关节', self._add_joint)
        self._button(row, '删除选中关节', self._delete_joint)
        help_text = QLabel('留空=未知；q_home=主机零位对应的模型角；角色：policy / auxiliary / passive；策略序号从0连续；映射确认填“是/否”。')
        help_text.setWordWrap(True); row.addWidget(help_text, 1)
        model = QHBoxLayout(); box.addLayout(model)
        self.model_path = QLineEdit(); self.model_path.setPlaceholderText('选择结构包内的 robot.xml，原文件只读')
        self.model_path.textChanged.connect(self._profile_changed); model.addWidget(self.model_path, 1)
        self._button(model, '选择模型', self._choose_model)
        self._button(model, '只读预检', self._inspect)
        self.report_button = self._button(model, '导出预检报告', self._export_report)
        self.report_button.setEnabled(False)
        self.check_text = QPlainTextEdit(); self.check_text.setReadOnly(True); box.addWidget(self.check_text, 1)

    def _motion_tab(self):
        page = QWidget(); box = QVBoxLayout(page); self.tabs.addTab(page, '平顺轨迹预览')
        bar = QHBoxLayout(); box.addLayout(bar)
        self._button(bar, '打开位姿 JSON', self._load_poses)
        self._button(bar, '载入离线示例', self._demo)
        self.pose_source = QLabel('当前主界面动作库快照'); bar.addWidget(self.pose_source, 1)
        self.pose_table = QTableWidget(0, 3)
        self.pose_table.setHorizontalHeaderLabels(['动作名称', '最高速度 °/s', '结束暂停 s'])
        self.pose_table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        self.pose_table.itemChanged.connect(self._pose_edited)
        self.pose_table.setMaximumHeight(150); box.addWidget(self.pose_table)
        settings = QGroupBox('离线设计参数（示例值未经台架确认）'); controls = QHBoxLayout(settings); box.addWidget(settings)
        self.start = QComboBox(); self.start.addItem('首帧作为起点（不规划到首帧的运动）', 'first')
        if self.start_angles:
            self.start.addItem('打开时已标零反馈快照（不保证实时）', 'snapshot')
        self.start.currentIndexChanged.connect(self._invalidate_plan); controls.addWidget(self.start)
        self.accel = self._spin(controls, '加速度 °/s²', 25, .001, 1e6)
        self.jerk = self._spin(controls, '跃度 °/s³', 100, .001, 1e7)
        self.hz = self._spin(controls, '采样 Hz', 50, 1, 1000)
        self._button(controls, '生成平顺轨迹', self._plan)
        self.curves = CurveView(); box.addWidget(self.curves, 2)
        time_bar = QHBoxLayout(); box.addLayout(time_bar)
        self.joint = QComboBox(); self.joint.currentIndexChanged.connect(self._cursor); time_bar.addWidget(self.joint)
        self.scrub = QSlider(Qt.Orientation.Horizontal); self.scrub.setRange(0, 10000)
        self.scrub.valueChanged.connect(self._cursor); time_bar.addWidget(self.scrub, 1)
        self.cursor_label = QLabel('尚未规划'); time_bar.addWidget(self.cursor_label)
        self.motion_info = QPlainTextEdit(); self.motion_info.setReadOnly(True); self.motion_info.setMaximumHeight(110)
        box.addWidget(self.motion_info)
        export = QHBoxLayout(); box.addLayout(export)
        self.coordinate = QComboBox(); self.coordinate.addItem('主机角度参考草案（°）', False)
        self.coordinate.addItem('已确认映射的模型参考（rad）', True); export.addWidget(self.coordinate)
        self.export_button = self._button(export, '导出参考 JSON / CSV', self._export_plan)
        self.export_button.setEnabled(False)
        hint = QLabel('起点按静止设计；每帧停稳，暂停为0仍停稳。不做连续穿点；缺根部/身体轨迹，非完整训练片段。')
        hint.setWordWrap(True); box.addWidget(hint)

    def _spin(self, layout, label, default, low, high):
        layout.addWidget(QLabel(label)); spin = QDoubleSpinBox(); spin.setDecimals(3)
        spin.setRange(low, high); spin.setValue(default); spin.valueChanged.connect(self._invalidate_plan)
        layout.addWidget(spin); return spin

    def _error(self, exc):
        self.status.setText(str(exc))
        QMessageBox.warning(self, '请检查输入', str(exc))

    def _profile_changed(self, *args):
        if not self._filling:
            # 编辑映射后旧确认不得悄悄沿用；名称变化仍由整体哈希使报告失效。
            item = args[0] if args else None
            if isinstance(item, QTableWidgetItem) and item.column() != FIELDS.index('verified'):
                confirmed = self.table.item(item.row(), FIELDS.index('verified'))
                if confirmed is not None:
                    self._filling = True
                    confirmed.setText('否')
                    self._filling = False
            self.report = None
            if hasattr(self, 'report_button'): self.report_button.setEnabled(False)

    def _fill_profile(self, profile):
        self._filling = True
        self.profile_name.setText(profile.get('name', ''))
        self.table.setRowCount(len(profile['joints']))
        for row, joint in enumerate(profile['joints']):
            for col, key in enumerate(FIELDS):
                value = joint.get(key)
                text = ('是' if value else '否') if key == 'verified' else ('' if value is None else str(value))
                self.table.setItem(row, col, QTableWidgetItem(text))
        self._filling = False; self._profile_changed()

    def _current_profile(self):
        profile = deepcopy(self.profile)
        profile['name'] = self.profile_name.text().strip()
        joints = []
        for row in range(self.table.rowCount()):
            values = [self.table.item(row, col).text().strip() if self.table.item(row, col) else ''
                      for col in range(len(FIELDS))]
            joint = dict(zip(FIELDS, values))
            for key in ('motor_id', 'direction', 'policy_index'):
                joint[key] = int(joint[key]) if joint[key] else None
            joint['q_home_rad'] = float(joint['q_home_rad']) if joint['q_home_rad'] else None
            for key in ('semantic', 'model_joint', 'bus'):
                joint[key] = joint[key] or None
            if joint['verified'] not in ('是', '否', 'true', 'false'):
                raise ValueError(f'第 {row+1} 行映射确认请填写 是/否')
            joint['verified'] = joint['verified'] in ('是', 'true')
            joints.append(joint)
        profile['joints'] = joints
        rp.validate_profile(profile)
        return profile

    def _add_joint(self):
        row = self.table.rowCount()
        if row >= 128: return self._error('最多128个关节')
        self._filling = True; self.table.insertRow(row)
        for col in range(len(FIELDS)):
            self.table.setItem(row, col, QTableWidgetItem('否' if col == 8 else 'policy' if col == 6 else ''))
        self._filling = False; self._profile_changed()

    def _delete_joint(self):
        if self.table.currentRow() >= 0:
            self.table.removeRow(self.table.currentRow()); self._profile_changed()

    def _load_profile(self):
        path, _ = QFileDialog.getOpenFileName(self, '打开构型', '', 'JSON (*.json)')
        if not path: return
        try:
            profile = rp.read_json(path); rp.validate_profile(profile)
            self.profile = deepcopy(profile); self.baseline = deepcopy(profile); self._fill_profile(profile)
            self.status.setText('构型草案已打开；不改变当前电机映射。')
        except Exception as exc: self._error(exc)

    def _save_profile(self):
        try:
            profile = self._current_profile()
            path, _ = QFileDialog.getSaveFileName(self, '保存构型草案', 'robot_profile.json', 'JSON (*.json)')
            if path:
                rp.write_json(path, profile); self.status.setText('草案已保存：'+path)
        except Exception as exc: self._error(exc)

    def _check_profile(self):
        try:
            profile = self._current_profile(); checks = rp.validate_profile(profile)
            diff = rp.diff_profiles(self.baseline, profile)
            lines = [f'构型哈希：{rp.profile_digest(profile)}',
                     '相对打开时已变更，旧验证需重做。' if diff['validation_invalidated'] else '相对打开时无变化。']
            lines += [f'{c["code"]}：{c["message"]}' for c in checks]
            lines += [f'变更字段：{c["path"]}' for c in diff['changes']]
            self.check_text.setPlainText('\n'.join(lines) if lines else '结构字段完整；不代表物理认证')
        except Exception as exc: self._error(exc)

    def _choose_model(self):
        path, _ = QFileDialog.getOpenFileName(self, '选择 MJCF', '', 'MJCF (*.xml)')
        if path: self.model_path.setText(path)

    def _inspect(self):
        try:
            profile = self._current_profile(); path = self.model_path.text().strip()
            if not path: raise ValueError('请先选择模型 XML')
            self.report = None; self.report_button.setEnabled(False)
            self._task('model', lambda: inspect_model(path, profile))
        except Exception as exc: self._error(exc)

    def _export_report(self):
        if self.report is None: return
        path, _ = QFileDialog.getSaveFileName(self, '导出预检', 'model_preflight.json', 'JSON (*.json)')
        if path:
            report = deepcopy(self.report)
            self._task('export', lambda: rp.write_json(path, report))

    def _fill_poses(self):
        self._filling = True; self.pose_table.setRowCount(len(self.poses))
        for row, pose in enumerate(self.poses):
            for col, key in enumerate(('name', 'speed', 'dwell')):
                self.pose_table.setItem(row, col, QTableWidgetItem(str(pose.get(key, 12.5 if key == 'speed' else 1))))
        self._filling = False; self._invalidate_plan()

    def _pose_edited(self, item):
        if self._filling: return
        self._invalidate_plan()

    def _current_poses(self):
        poses = deepcopy(self.poses)
        for row, pose in enumerate(poses):
            pose['name'] = self.pose_table.item(row, 0).text().strip()
            pose['speed'] = float(self.pose_table.item(row, 1).text())
            pose['dwell'] = float(self.pose_table.item(row, 2).text())
        return poses

    def _load_poses(self):
        path, _ = QFileDialog.getOpenFileName(self, '打开位姿库', '', 'JSON (*.json)')
        if not path: return
        try:
            data = rp.read_json(path)
            if not isinstance(data, dict) or data.get('schema_version', 1) not in (1,2,3):
                raise ValueError('需要原上位机位姿库 JSON v1/v2/v3')
            if data.get('angle_unit', 'deg') != 'deg' or data.get('coordinate_frame', 'host_zero_relative') != 'host_zero_relative':
                raise ValueError('位姿需为主机零位相对角，单位为度')
            if data.get('speed_unit', 'deg/s') != 'deg/s' or data.get('pause_unit', 's') != 's':
                raise ValueError('位姿速度/暂停单位不匹配')
            poses = data['poses']
            if not isinstance(poses, list) or not 1 <= len(poses) <= 1000:
                raise ValueError('位姿库需要1–1000帧')
            legacy = False
            for pose in poses:
                if not isinstance(pose, dict) or not isinstance(pose.get('angles'), dict):
                    raise ValueError('位姿帧缺少angles')
                if 'speed' not in pose:
                    pose['speed'] = 12.5; legacy = True
            self.poses = poses; self._fill_poses(); self.pose_source.setText(path)
            self.status.setText('旧文件缺少速度，按12.5°/s迁移，请逐帧核对。' if legacy else '已载入离线位姿；不会发送电机。')
        except Exception as exc: self._error(exc)

    def _demo(self):
        self.poses = [dict(name='示例起点', angles={7:0,8:0}, speed=12.5, dwell=.5),
                      dict(name='示例转向', angles={7:12,8:-20}, speed=12.5, dwell=1),
                      dict(name='示例返回', angles={7:0,8:0}, speed=12.5, dwell=.5)]
        self.start.setCurrentIndex(0); self._fill_poses()
        self.pose_source.setText('人工离线示例 · 非实机验证动作')

    def _invalidate_plan(self, *args):
        self.plan = None
        if hasattr(self, 'export_button'): self.export_button.setEnabled(False)
        if hasattr(self, 'curves'): self.curves.plan = None; self.curves.update()
        if hasattr(self, 'motion_info'): self.motion_info.setPlainText('输入已更新，请重新生成轨迹。')

    def _plan(self):
        try:
            poses = self._current_poses()
            start = deepcopy(self.start_angles) if self.start.currentData() == 'snapshot' else None
            accel, jerk, hz = self.accel.value(), self.jerk.value(), self.hz.value()
            self._invalidate_plan()
            self._task('plan', lambda: plan_poses(poses, start, accel, jerk, hz, self._cancel))
        except Exception as exc: self._error(exc)

    def _cursor(self, *args):
        if not self.plan: return
        i = max(0, self.joint.currentIndex())
        t = self.plan['duration_s']*self.scrub.value()/10000
        self.curves.joint = i; self.curves.cursor = t; self.curves.update()
        times = [s['t_s'] for s in self.plan['samples']]
        s = self.plan['samples'][min(len(times)-1, bisect_left(times, t))]
        self.cursor_label.setText(f'{t:.3f}s  角度 {s["q_deg"][i]:.3f}°  速度 {s["dq_deg_s"][i]:.3f}°/s')

    def _export_plan(self):
        if self.plan is None: return
        try:
            profile = self._current_profile()
            path, _ = QFileDialog.getSaveFileName(self, '导出参考草案', 'motion_reference.json', 'JSON (*.json);;CSV (*.csv)')
            if path:
                plan = deepcopy(self.plan); model = bool(self.coordinate.currentData())
                self._task('export', lambda: export_reference(path, plan, profile, model))
        except Exception as exc: self._error(exc)

    def _task(self, kind, function):
        if self._thread or self._closing: return
        self._cancel.clear(); self._result = None
        self.tabs.setEnabled(False)
        # 原子导出不能在替换目标文件中途取消；关闭窗口同样等它收尾。
        self.cancel_button.setEnabled(kind != 'export')
        self.status.setText('正在写入文件，请等待完成…' if kind == 'export' else '正在处理离线任务…')
        def run():
            try: self._result = (kind, function(), None)
            except Exception as exc: self._result = (kind, None, exc)
        self._thread = threading.Thread(target=run, daemon=True); self._thread.start()

    def _poll(self):
        if self._thread and not self._thread.is_alive():
            kind, value, error = self._result
            self._thread = None; self._result = None
            self.tabs.setEnabled(True); self.cancel_button.setEnabled(False)
            if self._closing:
                self.done(self._close_result); return
            if self._cancel.is_set() and kind != 'export':
                self.status.setText('任务已取消，未保留部分结果。'); return
            if error:
                self.status.setText(str(error)); return
            if kind == 'model':
                self.report = value; self.report_button.setEnabled(True)
                lines = [f'状态：{value["status"]} · 仅根XML静态检查',
                         f'显式质量和：{value.get("explicit_mass_sum", "未知")}（单位须核实）',
                         f'XML SHA256：{value["sha256"]}']
                lines += [f'[{c["level"]}] {c["message"]}' for c in value['checks']]
                self.check_text.setPlainText('\n'.join(lines))
            elif kind == 'plan':
                self.plan = value; self.curves.plan = value
                self.joint.clear()
                for mid in value['joint_ids']: self.joint.addItem(f'ID {mid}')
                lines = [f'Ruckig 本地同步 · {value["duration_s"]:.3f}s · {len(value["samples"])}采样点',
                         '速度为上限；每帧结束速度/加速度为零；跃度有界但可跳变。']
                lines += [f'{s["name"]}：运动 {s["motion_end_s"]-s["start_s"]:.3f}s，暂停 {s["end_s"]-s["motion_end_s"]:.3f}s'
                          for s in value['segments']]
                self.motion_info.setPlainText('\n'.join(lines)); self.export_button.setEnabled(True); self._cursor()
            self.status.setText('导出完成。' if kind == 'export' else '离线任务完成；未发送电机命令。')

    def _defer_close(self, result):
        """QDialog 的 Esc/reject/done 都要等待后台收尾，不能只处理窗口叉号。"""
        self._close_result = result
        if self._thread:
            self._closing = True; self._cancel.set()
            self.cancel_button.setEnabled(False)
            self.status.setText('正在等待离线任务收尾，写入中的文件会保存完成…')
            return True
        self.timer.stop()
        return False

    def done(self, result):
        if not self._defer_close(result):
            super().done(result)

    def reject(self):
        self.done(QDialog.DialogCode.Rejected)

    def closeEvent(self, event):
        if self._defer_close(QDialog.DialogCode.Rejected):
            event.ignore()
        else:
            event.accept()
            super().closeEvent(event)
