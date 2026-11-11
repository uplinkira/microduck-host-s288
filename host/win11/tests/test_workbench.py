"""Qt 离线工作台回归：不枚举/连接真实串口，不发送运动命令。"""
import json
import os
from pathlib import Path
import sys
import tempfile
import threading
import time
import unittest
from unittest.mock import patch

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from PyQt6.QtCore import Qt
from PyQt6 import sip
from PyQt6.QtTest import QTest
from PyQt6.QtWidgets import QApplication, QDialog
import robot_profile as rp
import workbench_ui as wb
import pose_core as pc
import pose_ui as ui


class WorkbenchTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def setUp(self):
        self.folder = tempfile.TemporaryDirectory(prefix='xianduck_workbench_')
        self.addCleanup(self.folder.cleanup)
        self.dialogs = []
        self.workers = []
        self.warning = patch.object(wb.QMessageBox, 'warning').start()
        self.addCleanup(patch.stopall)

    def tearDown(self):
        for gate, thread in self.workers:
            gate.set()
            thread.join(2)
        for dialog in self.dialogs:
            if sip.isdeleted(dialog):
                continue
            if dialog._thread:
                dialog._cancel.set()
                dialog._thread.join(2)
                dialog._poll()
            dialog.close()
        self.app.processEvents()

    def dialog(self):
        profile = rp.draft_from_hardware({7: {'joint': 'head_pitch'}, 8: {'joint': 'head_yaw'}})
        dialog = wb.WorkbenchDialog(None, profile, [])
        self.dialogs.append(dialog)
        dialog.show()
        self.app.processEvents()
        return dialog, profile

    def until(self, condition, timeout=5):
        deadline = time.monotonic() + timeout
        while not condition() and time.monotonic() < deadline:
            self.app.processEvents()
            QTest.qWait(10)
        self.assertTrue(condition(), '异步 UI 条件未在期限内满足')

    def blocked_task(self, dialog, kind='plan'):
        entered, release = threading.Event(), threading.Event()
        def run():
            entered.set()
            if not release.wait(3):
                raise RuntimeError('测试后台释放超时')
            return {'discard_this': True}
        dialog._task(kind, run)
        self.assertTrue(entered.wait(1))
        thread = dialog._thread
        self.workers.append((release, thread))
        return release, thread

    def test_draft_fields_stay_unknown_and_inputs_are_copies(self):
        dialog, original = self.dialog()
        current = dialog._current_profile()
        for joint in current['joints']:
            for field in ('direction', 'q_home_rad', 'model_joint', 'bus', 'policy_index'):
                self.assertIsNone(joint[field])
            self.assertFalse(joint['verified'])
        dialog.profile_name.setText('新构型')
        self.assertEqual(original['name'], 'xianduck_s288')
        self.assertEqual(dialog.baseline['name'], 'xianduck_s288')
        self.assertEqual(dialog._current_profile()['name'], '新构型')

    def test_demo_plans_and_exports_explicit_nontraining_reference(self):
        dialog, _ = self.dialog()
        dialog._demo()
        dialog._plan()
        self.until(lambda: dialog._thread is None)
        self.assertIsNotNone(dialog.plan)
        self.assertTrue(dialog.export_button.isEnabled())
        self.assertGreater(dialog.plan['duration_s'], 0)
        target = Path(self.folder.name) / '参考.json'
        with patch.object(wb.QFileDialog, 'getSaveFileName', return_value=(str(target), 'JSON')):
            dialog._export_plan()
        self.until(lambda: dialog._thread is None)
        data = json.loads(target.read_text(encoding='utf-8'))
        self.assertEqual(data['data_kind'], 'planned_joint_reference')
        self.assertFalse(data['training_ready'])
        self.assertFalse(data['hardware_validated'])
        self.assertEqual(data['coordinate_frame'], 'host_zero_relative')
        self.assertTrue(any(c['code'] == 'missing_direction' for c in data['profile_checks']))
        self.warning.assert_not_called()

    def test_async_task_disables_inputs_and_cancel_discards_plan(self):
        dialog, _ = self.dialog()
        release, _ = self.blocked_task(dialog)
        for widget in (dialog.profile_name, dialog.table, dialog.pose_table, dialog.accel,
                       dialog.jerk, dialog.hz, dialog.start, dialog.coordinate):
            self.assertFalse(widget.isEnabled())
        self.assertTrue(dialog.cancel_button.isEnabled())
        QTest.mouseClick(dialog.cancel_button, Qt.MouseButton.LeftButton)
        self.assertTrue(dialog._cancel.is_set())
        release.set()
        self.until(lambda: dialog._thread is None)
        self.assertIsNone(dialog.plan)
        self.assertFalse(dialog.export_button.isEnabled())
        self.assertTrue(dialog.tabs.isEnabled())

    def test_all_close_routes_wait_for_background_completion(self):
        for route in ('reject', 'escape', 'done', 'close'):
            with self.subTest(route=route):
                dialog, _ = self.dialog()
                release, thread = self.blocked_task(dialog)
                if route == 'escape':
                    QTest.keyClick(dialog, Qt.Key.Key_Escape)
                elif route == 'done':
                    dialog.done(QDialog.DialogCode.Rejected)
                else:
                    getattr(dialog, route)()
                self.app.processEvents()
                try:
                    self.assertTrue(dialog.isVisible(), '后台未结束时窗口不能直接隐藏/销毁')
                    self.assertTrue(dialog._cancel.is_set(), '关闭必须发出取消')
                    self.assertTrue(thread.is_alive())
                finally:
                    dialog._cancel.set()
                    release.set()
                self.until(lambda: dialog._thread is None)
                self.until(lambda: not dialog.isVisible())
                self.assertFalse(thread.is_alive())
                self.assertFalse(dialog.timer.isActive())
                self.assertIsNone(dialog.plan)

    def test_mapping_edit_revokes_confirmation_and_report(self):
        dialog, _ = self.dialog()
        verified_col = wb.FIELDS.index('verified')
        direction_col = wb.FIELDS.index('direction')
        dialog.table.item(0, verified_col).setText('是')
        dialog.table.item(1, verified_col).setText('是')
        dialog.report = {'status': 'review_required'}
        dialog.report_button.setEnabled(True)
        dialog.table.item(0, direction_col).setText('-1')
        self.assertEqual(dialog.table.item(0, verified_col).text(), '否')
        self.assertEqual(dialog.table.item(1, verified_col).text(), '是')
        self.assertIsNone(dialog.report)
        self.assertFalse(dialog.report_button.isEnabled())

    def test_export_cannot_cancel_and_close_waits_for_write(self):
        dialog, _ = self.dialog()
        entered, release, completed = threading.Event(), threading.Event(), threading.Event()
        target = Path(self.folder.name) / 'export.json'
        def export():
            entered.set()
            if not release.wait(3):
                raise RuntimeError('测试释放超时')
            rp.write_json(target, {'finished': True})
            completed.set()
        dialog._task('export', export)
        self.assertTrue(entered.wait(1))
        self.workers.append((release, dialog._thread))
        self.assertFalse(dialog.cancel_button.isEnabled())
        QTest.mouseClick(dialog.cancel_button, Qt.MouseButton.LeftButton)
        self.assertFalse(dialog._cancel.is_set())
        dialog.reject()
        self.assertTrue(dialog.isVisible())
        self.assertFalse(completed.is_set())
        release.set()
        self.until(lambda: dialog._thread is None and not dialog.isVisible())
        self.assertTrue(completed.is_set())
        self.assertEqual(rp.read_json(target), {'finished': True})

    def test_main_close_waits_for_workbench_export(self):
        with patch.object(pc, 'CALIB_FILE', str(Path(self.folder.name) / 'calib.json')), \
                patch.object(pc, 'POSE_FILE', str(Path(self.folder.name) / 'poses.json')), \
                patch.object(ui, 'list_ports', return_value=[]), \
                patch.object(pc.su, 'Bus') as bus:
            ctrl = pc.PoseController(None, 6000000, True, [7, 8])
            main = ui.Main(ctrl)
            main.show()
            main._open_workbench()
            dialog = main._workbench_dialog
            self.dialogs.append(dialog)
            entered, release, completed = threading.Event(), threading.Event(), threading.Event()
            target = Path(self.folder.name) / 'closing_export.json'
            def export():
                entered.set()
                if not release.wait(3):
                    raise RuntimeError('测试释放超时')
                rp.write_json(target, {'finished': True})
                completed.set()
            dialog._task('export', export)
            thread = dialog._thread
            self.workers.append((release, thread))
            try:
                self.assertTrue(entered.wait(1))
                main.close()
                self.app.processEvents()
                self.assertTrue(main.isVisible(), '主窗口必须等待子工作台后台写入')
                self.assertTrue(dialog.isVisible())
                self.assertFalse(completed.is_set())
                self.assertTrue(thread.is_alive())
                release.set()
                self.until(lambda: not main.isVisible())
                self.assertTrue(completed.is_set())
                self.assertFalse(thread.is_alive())
                self.assertEqual(rp.read_json(target), {'finished': True})
                bus.assert_not_called()
            finally:
                release.set()
                thread.join(2)
                if not sip.isdeleted(dialog) and dialog._thread:
                    dialog._poll()
                main._allow_close = True
                main.close()
                ctrl.shutdown()

    def test_main_opens_offline_without_connection_or_motion(self):
        with patch.object(pc, 'CALIB_FILE', str(Path(self.folder.name) / 'calib.json')), \
                patch.object(pc, 'POSE_FILE', str(Path(self.folder.name) / 'poses.json')), \
                patch.object(ui, 'list_ports', return_value=[]), \
                patch.object(pc.su, 'Bus') as bus:
            ctrl = pc.PoseController(None, 6000000, True, [7, 8])
            main = ui.Main(ctrl)
            try:
                main.show()
                with patch.object(ctrl, 'connect') as connect, \
                        patch.object(ctrl, 'goto') as goto, \
                        patch.object(ctrl, 'set_enabled') as enable, \
                        patch.object(ctrl, 'transition') as transition:
                    self.assertTrue(hasattr(main, 'b_workbench'), '主界面缺少离线工作台入口')
                    main._open_workbench()
                    self.app.processEvents()
                    dialog = main._workbench_dialog
                    self.assertIsNotNone(dialog)
                    self.dialogs.append(dialog)
                    self.assertTrue(dialog.isVisible())
                    self.assertFalse(ctrl.connected)
                    connect.assert_not_called()
                    goto.assert_not_called()
                    enable.assert_not_called()
                    transition.assert_not_called()
                    bus.assert_not_called()
            finally:
                for dialog in self.dialogs:
                    dialog.close()
                main._allow_close = True
                main.close()
                ctrl.shutdown()


if __name__ == '__main__':
    unittest.main()
