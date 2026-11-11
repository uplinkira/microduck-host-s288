"""采集参数在GUI线程读取；工作线程只使用不可变配置，结果由主窗口轮询。"""
import os
from PyQt6.QtCore import QTimer
from PyQt6.QtGui import QKeySequence, QShortcut
from PyQt6.QtWidgets import (QDialog,QVBoxLayout,QFormLayout,QWidget,QScrollArea,QLabel,
                            QComboBox,QSpinBox,QDoubleSpinBox,QLineEdit,QCheckBox,QPushButton,QMessageBox)
from measurement import MeasurementConfig, TRAJECTORIES
import pose_core as pc


class MeasurementDialog(QDialog):
    def __init__(self,parent,ctrl):
        super().__init__(parent)
        self.owner,self.ctrl=parent,ctrl
        self.setWindowTitle('S288 实测采集'); self.resize(660,750)
        layout=QVBoxLayout(self); scroll=QScrollArea(); scroll.setWidgetResizable(True)
        form_widget=QWidget(); form=QFormLayout(form_widget); scroll.setWidget(form_widget); layout.addWidget(scroll)
        self.fields=form_widget
        self.kind=QComboBox()
        for title,data in [('单台往返（零增益）','latency'),('全部非嘴ID整轮往返（零增益）','round'),('单台纯位置阶跃','position'),('单台摆锤轨迹/九组','trajectory')]:
            self.kind.addItem(title,data)
        self.mid=QComboBox()
        for m in ctrl.states:self.mid.addItem(f'ID {m} · {ctrl.states[m].spec["name"]}',m)
        self.cycles=QSpinBox(); self.cycles.setRange(1,10000); self.cycles.setValue(1000)
        self.duration=QDoubleSpinBox(); self.duration.setRange(.1,60); self.duration.setValue(5)
        self.hz=QDoubleSpinBox(); self.hz.setRange(1,200); self.hz.setValue(50)
        self.gains=QLineEdit('3'); self.gains.setPlaceholderText('逗号分隔，最多三档；不是XL330固件值')
        self.kd=QDoubleSpinBox(); self.kd.setRange(0,21.2); self.kd.setDecimals(3); self.kd.setValue(.3)
        self.amplitude=QDoubleSpinBox(); self.amplitude.setRange(-5,5); self.amplitude.setValue(3)
        self.traj=QComboBox(); self.traj.addItem('全部三条（每条6秒）','all')
        for n in TRAJECTORIES:self.traj.addItem(n,n)
        self.mass=QLineEdit(); self.arm_mass=QLineEdit(); self.length=QLineEdit()
        self.direction=QComboBox(); self.direction.addItem('未确认',None); self.direction.addItem('+1',1); self.direction.addItem('-1',-1)
        self.operator=QLineEdit(); self.firmware=QLineEdit(); self.fixture=QLineEdit()
        self.torque=QDoubleSpinBox(); self.torque.setRange(.01,.6); self.torque.setValue(.6)
        self.confirm=QCheckBox('已固定台架、确认零位/方向/增益/运动范围；摆锤零位竖直向下，均匀杆与长度模型一致')
        for title,field in [('类型',self.kind),('单台ID',self.mid),('往返次数/整轮次数',self.cycles),
                            ('阶跃每组时长(s)',self.duration),('主动采样目标频率(Hz)',self.hz),
                            ('Kp (N·m/rad)',self.gains),('Kd (N·m·s/rad)',self.kd),('阶跃幅度(°，相对已标定零位)',self.amplitude),
                            ('摆锤轨迹',self.traj),('配重质量(kg)',self.mass),('均匀杆质量(kg)',self.arm_mass),('转轴至配重重心/杆长(m)',self.length),
                            ('摆锤正方向相对电机输出',self.direction),('力矩反馈停止阈值(N·m)',self.torque),
                            ('操作者',self.operator),('电机固件版本',self.firmware),('台架/零位/负载/电压说明',self.fixture)]:form.addRow(title,field)
        form.addRow(self.confirm)
        info=QLabel('原始数据不是拟合参数。主动轨迹不经过普通位姿限速；首末回零为10°/s准备过程。摆锤最高±90°，LiftAndDrop含零增益自由下落。取消/故障立即零增益，不返回零位。'); info.setWordWrap(True); form.addRow(info)
        self.out=QLabel('结果目录：EXE旁 measurement/；JSON、CSV、逐组数据和时序统计。'); self.out.setWordWrap(True); layout.addWidget(self.out)
        self.start=QPushButton('开始采集'); self.start.clicked.connect(self._start); layout.addWidget(self.start)
        self.cancel=QPushButton('停止并卸力'); self.cancel.clicked.connect(self._stop); layout.addWidget(self.cancel)
        QShortcut(QKeySequence('Escape'),self).activated.connect(self._stop)
        self.timer=QTimer(self); self.timer.timeout.connect(self._tick); self.timer.start(100)

    def _config(self):
        kind=self.kind.currentData(); mid=self.mid.currentData()
        ids=tuple(m for m in self.ctrl.states if m not in (0,14)) if kind=='round' else (mid,)
        if kind=='round' and len(ids)!=13:raise ValueError('整轮测试要求完整13个非嘴ID；重新连接完整ID集合')
        is_traj=kind=='trajectory'; active=kind in ('position','trajectory')
        number=lambda field:float(field.text()) if field.text().strip() else None
        return MeasurementConfig(kind='latency' if kind=='round' else kind,ids=ids,cycles=self.cycles.value(),
                                 duration_s=self.duration.value(),hz=self.hz.value(),
                                 gains=tuple(float(x.strip()) for x in self.gains.text().split(',')) if active else (3.,),
                                 kd=self.kd.value(),amplitude_deg=self.amplitude.value(),
                                 trajectories=TRAJECTORIES if self.traj.currentData()=='all' else (self.traj.currentData(),),
                                 mass=number(self.mass) if is_traj else None,arm_mass=number(self.arm_mass) if is_traj else None,
                                 length=number(self.length) if is_traj else None,
                                 pendulum_direction=self.direction.currentData() if is_traj else 1,
                                 fixture_confirmed=self.confirm.isChecked(),operator=self.operator.text(),
                                 firmware=self.firmware.text(),fixture=self.fixture.text(),torque_limit=self.torque.value()).validate()

    def _start(self):
        try:
            if self.owner._busy() or self.owner.lib.playing():raise ValueError('请等待维护任务或播放结束')
            cfg=self._config()
            if cfg.kind!='latency' and QMessageBox.question(self,'确认主动运动',
                '将发送未经普通位姿限速的试验目标。确认台架、固件单位与退出时零增益卸力均已检查？')!=QMessageBox.StandardButton.Yes:return
            self.owner.b_log.setChecked(False)
            self.ctrl.reserve_measurement(cfg)
            if not self.owner._task('实测采集',lambda:self.ctrl.collect_measurement(cfg,os.path.join(pc.DATA_DIR,'measurement'))):
                self.ctrl.measurement_active=False; raise RuntimeError('未能启动维护任务')
            self.out.setText('正在采集；详细路径和结果在主窗口状态栏。'); self._tick()
        except Exception as exc:
            if self.ctrl.measurement_active and not self.owner._busy():
                self.ctrl.measurement_stop.set()
                self.ctrl.measurement_active=False
            self.out.setText(str(exc))

    def _stop(self):
        self.ctrl.release_all(); self.out.setText('已请求停止；正在卸力并保存，结果见主窗口状态栏。')

    def _tick(self):
        active=self.ctrl.measurement_active
        self.fields.setEnabled(not active and not self.owner._busy())
        self.start.setEnabled(not active and not self.owner._busy())
        self.cancel.setEnabled(active)
        if active and self.ctrl.measurement_runner:self.out.setText(self.ctrl.measurement_runner.progress or '正在准备采集…')

    def reject(self):
        if self.ctrl.measurement_active:self._stop()
        else:super().reject()

    def closeEvent(self,event):
        if self.ctrl.measurement_active:
            self._stop(); event.ignore()
        else:event.accept()
