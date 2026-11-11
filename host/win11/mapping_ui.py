"""人工微动后的语义/方向实测报告，不修改电机ID、不冒充自动检测。"""
import csv
import math
from pathlib import Path
from PyQt6.QtWidgets import (QDialog,QVBoxLayout,QLabel,QLineEdit,QTableWidget,QTableWidgetItem,
                            QPushButton,QFileDialog,QMessageBox)
import pose_core as pc

FIELDS=('motor_id','expected_joint','observed_joint','direction','q_home_rad','observation')

def validate_rows(rows):
    for r in rows:
        if r['direction'] not in ('','+1','1','-1'):raise ValueError('direction只能为空、+1或-1；空表示未确认')
        if r['q_home_rad'] and not math.isfinite(float(r['q_home_rad'])):raise ValueError('q_home必须为空或有限弧度数值')
    return rows


class MappingDialog(QDialog):
    def __init__(self,parent,ctrl):
        super().__init__(parent); self.ctrl=ctrl
        self.setWindowTitle('实机映射与方向记录'); self.resize(980,620)
        layout=QVBoxLayout(self)
        info=QLabel('先小幅发送“电机输出正向”再观察；未知不填。模型q_home须算法确认，不能照抄旧STAND。此表只导出证据，不自动改运行映射或电机ID。')
        info.setWordWrap(True); layout.addWidget(info)
        self.operator=QLineEdit(); self.operator.setPlaceholderText('操作者（必填）'); layout.addWidget(self.operator)
        self.table=QTableWidget(len(ctrl.states),len(FIELDS)); self.table.setHorizontalHeaderLabels(FIELDS); layout.addWidget(self.table)
        for row,(mid,st) in enumerate(ctrl.states.items()):
            vals=(str(mid),st.spec.get('joint') or '', '', '', '', '')
            for col,val in enumerate(vals):
                item=QTableWidgetItem(val)
                if col<2:
                    from PyQt6.QtCore import Qt
                    item.setFlags(item.flags() & ~Qt.ItemFlag.ItemIsEditable)
                self.table.setItem(row,col,item)
        button=QPushButton('导出实测表 CSV + JSON'); button.clicked.connect(self._save); layout.addWidget(button)

    def _save(self):
        try:
            if not self.operator.text().strip():raise ValueError('请填写操作者')
            rows=validate_rows([dict(zip(FIELDS,[self.table.item(r,c).text().strip() for c in range(len(FIELDS))])) for r in range(self.table.rowCount())])
            path,_=QFileDialog.getSaveFileName(self,'导出实测记录','joint_measurements.csv','CSV (*.csv)')
            if not path:return
            p=Path(path)
            with p.open('w',encoding='utf-8-sig',newline='') as f:
                writer=csv.DictWriter(f,fieldnames=FIELDS); writer.writeheader(); writer.writerows(rows)
            from datetime import datetime
            pc.atomic_json(str(p)+'.json',dict(schema='s288_joint_report_v1',operator=self.operator.text(),
                           when=datetime.now().isoformat(),rows=rows,calibration=self.ctrl.calib,
                           note='人工实测证据；空字段未确认；不自动修改运行时JOINTS或方向'))
            QMessageBox.information(self,'已保存',str(p))
        except Exception as exc:QMessageBox.warning(self,'导出失败',str(exc))
