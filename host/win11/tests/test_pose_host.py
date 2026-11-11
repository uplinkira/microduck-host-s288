# -*- coding: utf-8 -*-
"""离线回归：假总线与仿真，不枚举/打开真实串口，不替代实机验收。"""
import csv
import json
import math
import os
from pathlib import Path
import sys
import tempfile
import time
import threading
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
import pose_core as pc
import pose_ui as ui
from PyQt6.QtWidgets import QApplication


class CoreTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix='xianduck_test_')
        self.folder = Path(self.temp.name)
        self.patches = [patch.object(pc, 'CALIB_FILE', str(self.folder / 'calib.json')),
                        patch.object(pc, 'POSE_FILE', str(self.folder / 'poses.json'))]
        for p in self.patches:
            p.start()
        self.ctrl = pc.PoseController(None, 6000000, True, [7, 8])

    def tearDown(self):
        self.ctrl.shutdown()
        for p in reversed(self.patches):
            p.stop()
        self.temp.cleanup()

    def test_disconnect_reconnect_clears_enabled_feedback(self):
        for _ in range(3):
            self.assertTrue(self.ctrl.connect())
            self.assertTrue(self.ctrl.zero_now(7))
            self.assertTrue(self.ctrl.set_enabled(7, True))
            self.ctrl.shutdown()
            self.assertFalse(self.ctrl.connected)
            self.assertIsNone(self.ctrl.thread)
            self.assertTrue(all(not s.enabled and s.fb is None for s in self.ctrl.states.values()))
        self.assertTrue(self.ctrl.connect())
        self.assertTrue(self.ctrl.states[7].centered)
        self.assertFalse(self.ctrl.states[7].enabled)

    def test_enable_holds_measured_position_and_estop_latches(self):
        self.ctrl.connect()
        self.ctrl.zero_now(7)
        self.ctrl.states[7].fb['outputPos'] = math.radians(8)
        self.assertTrue(self.ctrl.set_enabled(7, True))
        self.assertAlmostEqual(math.degrees(self.ctrl.states[7].cmd), 8)
        self.ctrl.release_all()
        self.ctrl.goto({7: 3})
        self.assertFalse(self.ctrl.states[7].enabled)

    def test_zero_rejects_missing_fault_and_motion(self):
        self.ctrl.connect()
        base = dict(outputPos=0, ExPos=0, MError=0, Temp=30, vol=12, outputTor=0)
        for samples in ([base, None], [dict(base, MError=1)],
                        [base, dict(base, outputPos=.1), base]):
            with patch.object(self.ctrl, '_probe', side_effect=samples):
                self.assertFalse(self.ctrl.zero_now(7))
        self.assertFalse(self.ctrl.states[7].centered)
        self.assertFalse(Path(pc.CALIB_FILE).exists())

    def test_atomic_failure_preserves_old_calibration(self):
        self.ctrl.connect()
        self.ctrl.zero_now(7)
        old = Path(pc.CALIB_FILE).read_bytes()
        with patch.object(pc.os, 'replace', side_effect=PermissionError('locked')):
            with self.assertRaises(PermissionError):
                self.ctrl.zero_now(8)
        self.assertNotIn(8, self.ctrl.calib)
        self.assertFalse(self.ctrl.states[8].centered)
        self.assertEqual(old, Path(pc.CALIB_FILE).read_bytes())

    def test_json_csv_roundtrip_and_invalid_load_is_atomic(self):
        lib = pc.PoseLibrary(self.ctrl)
        lib.poses = [dict(name='将军立,示教\n第二行', trans=1.5, dwell=1, angles={7: 2.5})]
        jp, cp = self.folder / '姿态.json', self.folder / '姿态.csv'
        lib.save(jp)
        lib.export_csv(cp)
        self.assertIn('将军立', jp.read_text(encoding='utf-8'))
        with cp.open(encoding='utf-8-sig', newline='') as f:
            rows = list(csv.reader(f))
        self.assertEqual(rows[0], ['name', 'speed_deg_s', 'dwell_s', 'id_7_deg', 'id_8_deg'])
        self.assertEqual(rows[1][-1], '')
        self.assertEqual(rows[1][0], lib.poses[0]['name'])
        lib.load(jp)
        saved = lib.poses.copy()
        for bad in (float('nan'), float('inf'), 1e9):
            data = {'poses': [dict(name='坏数据', angles={'7': bad})]}
            jp.write_text(json.dumps(data, ensure_ascii=False), encoding='utf-8')
            with self.assertRaises(ValueError):
                lib.load(jp)
            self.assertEqual(saved, lib.poses)
        jp.write_text(json.dumps({'poses': [dict(name='旧格式', angles={'7': 2})]}, ensure_ascii=False), encoding='utf-8')
        lib.load(jp)
        self.assertEqual(lib.poses[0]['angles'], {7: 2.0})

    def test_failed_serial_connection_can_retry_and_release(self):
        class FakeBus:
            instances = []
            def __init__(self, port, baud):
                self.closed = False
                self.releases = []
                self.instances.append(self)
            def probe(self, mid):
                return dict(outputPos=0, Temp=30, vol=12, outputTor=0, MError=0)
            def release(self, mid, kd):
                self.releases.append(mid)
            def close(self):
                self.closed = True
        self.ctrl.sim = False
        with patch.object(pc.su, 'Bus', side_effect=OSError('could not open port')):
            self.assertFalse(self.ctrl.connect())
        with patch.object(pc.su, 'Bus', FakeBus):
            self.assertTrue(self.ctrl.connect())
            self.ctrl.refresh_offline()
            self.ctrl.shutdown()
        self.assertTrue(FakeBus.instances[0].closed)
        self.assertEqual(FakeBus.instances[0].releases, [7, 8])

    def test_no_angle_clipping_and_playback_halts(self):
        self.ctrl.connect()
        self.ctrl.zero_now(7)
        self.ctrl.ranges[7] = [-8, 8]  # 历史行程只做记录，不能再裁剪。
        lib = pc.PoseLibrary(self.ctrl)
        lib.poses = [dict(name='测试', trans=.1, dwell=0, angles={7: 60})]
        lib.set_angle(0, 7, 270)
        self.assertEqual(lib.poses[0]['angles'][7], 270)
        p = self.folder / 'wide.json'
        lib.save(p); lib.load(p)
        self.assertEqual(lib.poses[0]['angles'][7], 270)
        self.ctrl.goto({7: 60})
        self.assertAlmostEqual(math.degrees(self.ctrl.states[7].target), 60)
        self.ctrl.transition({7: 60}, .1, threading.Event(), peak_limit=10000)
        self.assertAlmostEqual(math.degrees(self.ctrl.states[7].target), 60)
        self.assertTrue(lib.play(0, True))
        lib.stop.set()
        self.ctrl.release_all()
        lib.halt()
        self.assertFalse(lib.playing())
        self.assertFalse(self.ctrl.states[7].enabled)
        self.ctrl.range_rec = True
        self.assertFalse(self.ctrl.set_enabled(7, True))
        self.ctrl.range_rec = False

    def test_multiturn_feedback_and_protocol_boundary(self):
        self.ctrl.connect(); self.ctrl.zero_now(7)
        self.ctrl.states[7].fb['outputPos'] = math.radians(270)
        self.assertEqual(self.ctrl.read_angles(True)[7], 270)
        self.assertTrue(self.ctrl.set_enabled(7, True))
        self.assertAlmostEqual(math.degrees(self.ctrl.states[7].cmd), 270)
        self.assertFalse(self.ctrl.set_target(7, 1e9))
        self.assertAlmostEqual(math.degrees(self.ctrl.states[7].target), 270)

    def test_playback_speed_pause_and_manual_speed_restoration(self):
        self.ctrl.connect(); self.ctrl.zero_now(7)
        lib = pc.PoseLibrary(self.ctrl)
        lib.poses = [dict(name='动作', speed=20, dwell=.15, angles={7:2})]
        before = self.ctrl.speed
        started = time.monotonic()
        self.assertTrue(lib.play(1, False)); lib.thread.join(3)
        self.assertFalse(lib.playing()); self.assertIn('播放结束', lib.msg)
        self.assertGreaterEqual(time.monotonic()-started, .15)
        self.assertEqual(self.ctrl.speed, before)
        self.assertAlmostEqual(math.degrees(self.ctrl.states[7].fb['outputPos']-self.ctrl.states[7].center),2,delta=.5)
        with self.assertRaises(ValueError):lib.set_timing(0,speed=0)
        with self.assertRaises(ValueError):lib.set_timing(0,dwell=-1)

    def test_legacy_migration_is_explicit_and_preserves_old_duration(self):
        path=self.folder/'legacy.json'
        path.write_text(json.dumps({'poses':[dict(name='旧动作',trans=2,dwell=3,angles={'7':10})]}),encoding='utf-8')
        lib=pc.PoseLibrary(self.ctrl); lib.load(path)
        self.assertIn('迁移',lib.load_warning)
        self.assertEqual(lib.poses[0]['speed'],pc.SPEED_DEFAULT)
        self.assertEqual(lib.poses[0]['legacy_trans_s'],2)
        lib.save(path); lib.load(path)
        self.assertEqual(lib.load_warning,'')
        self.assertEqual(lib.poses[0]['dwell'],3)

    def test_fault_stops_before_following_action(self):
        self.ctrl.connect(); self.ctrl.zero_now(7)
        self.ctrl.states[7].error='测试故障'
        lib=pc.PoseLibrary(self.ctrl)
        lib.poses=[dict(name='故障帧',speed=10,dwell=0,angles={7:1}),dict(name='不得执行',speed=10,dwell=0,angles={7:20})]
        with patch.object(self.ctrl,'goto') as go:
            lib.play(1,False); lib.thread.join(2)
            go.assert_not_called()
        self.assertIn('播放失败',lib.msg)

    def test_fault_during_pause_stops_sequence(self):
        self.ctrl.connect(); self.ctrl.zero_now(7)
        lib = pc.PoseLibrary(self.ctrl)
        lib.poses = [dict(name='保持', speed=10, dwell=5, angles={7:0}),
                     dict(name='不得执行', speed=10, dwell=0, angles={7:20})]
        with patch.object(self.ctrl, 'goto', wraps=self.ctrl.goto) as go:
            lib.play(1)
            try:
                end = time.monotonic() + 2
                while '已到位' not in lib.msg and time.monotonic() < end:
                    time.sleep(.01)
                self.assertIn('已到位', lib.msg)
                with self.ctrl.lock:
                    self.ctrl.states[7].error = '暂停期间故障'
                lib.thread.join(1)
                self.assertFalse(lib.playing())
                self.assertIn('播放失败', lib.msg)
                self.assertEqual(go.call_count, 1)
            finally:
                lib.halt()

    def test_repeated_feedback_is_not_multiple_arrivals(self):
        self.ctrl.connect(); self.ctrl.zero_now(7)
        self.ctrl.running = False; self.ctrl.thread.join()
        lib = pc.PoseLibrary(self.ctrl)
        lib.poses = [dict(name='静止反馈', speed=10, dwell=0, angles={7:0})]
        try:
            lib.play(1); lib.thread.join(.2)
            self.assertTrue(lib.playing())
            self.assertNotIn('已到位', lib.msg)
        finally:
            lib.halt()

    def test_playback_uses_each_speed_and_pauses_before_next(self):
        self.ctrl.connect(); self.ctrl.zero_now(7)
        lib = pc.PoseLibrary(self.ctrl)
        lib.poses = [dict(name='慢', speed=10, dwell=.15, angles={7:2}),
                     dict(name='快', speed=20, dwell=0, angles={7:4})]
        calls = []
        real_goto = self.ctrl.goto
        def capture(angles):
            calls.append((self.ctrl.speed, time.monotonic()))
            return real_goto(angles)
        with patch.object(self.ctrl, 'goto', side_effect=capture):
            try:
                lib.play(1); lib.thread.join(3)
                self.assertFalse(lib.playing())
                self.assertIn('播放结束', lib.msg)
                self.assertEqual([v[0] for v in calls], [10, 20])
                self.assertGreaterEqual(calls[1][1]-calls[0][1], .3)
            finally:
                lib.halt()


class UiTests(unittest.TestCase):
    setUp = CoreTests.setUp
    tearDown = CoreTests.tearDown
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])
        ui.configure_font(cls.app)

    def wait_until(self, fn, timeout=5):
        end = time.monotonic() + timeout
        while not fn() and time.monotonic() < end:
            self.app.processEvents()
            time.sleep(.01)
        self.assertTrue(fn(), 'GUI 后台任务未在期限内完成')

    def test_window_ports_refresh_async_connect_disconnect(self):
        with patch.object(pc, 'port_details', return_value=[('COM2', 'USB A'), ('COM8', 'USB B')]):
            self.ctrl.sim = False
            win = ui.Main(self.ctrl)
            win.show()
            win.port_combo.setCurrentIndex(1)
            win._refresh_ports()
            self.assertEqual(win.port_combo.currentData(), 'COM8')
        with patch.object(pc, 'port_details', return_value=[]):
            win._refresh_ports()
            self.assertIsNone(win.port_combo.currentData())
        self.ctrl.sim = True
        win._refresh_ports()
        win._connection()
        self.wait_until(lambda: not win._busy())
        self.assertTrue(self.ctrl.connected)
        self.assertFalse(win.port_combo.isEnabled())
        self.assertNotIn('?', win.rows[7].w_name.text())
        win.rows[7].w_sp.setValue(270)
        self.assertAlmostEqual(math.degrees(self.ctrl.states[7].target), 270)
        self.assertGreaterEqual(win.rows[7].w_sl.maximum(), 540)
        win._connection()
        self.wait_until(lambda: not win._busy())
        self.assertFalse(self.ctrl.connected)
        win.close()
        self.wait_until(lambda: not win.isVisible())

    def test_chinese_width_and_screenshot(self):
        self._check_chinese_width_and_screenshot()

    def test_slider_range_controls_do_not_send_targets(self):
        self.ctrl.connect()
        win = ui.Main(self.ctrl)
        try:
            row = win.rows[7]
            self.assertEqual((row.w_sl.minimum(), row.w_sl.maximum()), (-360, 360))
            self.assertGreaterEqual(row.w_sl.minimumWidth(), 300)
            row.w_sl.setValue(180)
            self.assertAlmostEqual(math.degrees(self.ctrl.states[7].target), 90)
            with patch.object(self.ctrl, 'set_target', wraps=self.ctrl.set_target) as send:
                row.w_expand.click()
                send.assert_not_called()
            self.assertEqual(row.w_sl.maximum(), 720)
            self.assertEqual(row.w_sp.value(), 90)
            row.w_sl.setValue(540)
            self.assertAlmostEqual(math.degrees(self.ctrl.states[7].target), 270)
            with patch.object(self.ctrl, 'set_target', wraps=self.ctrl.set_target) as send:
                row.w_narrow.click()
                row.w_narrow.click()
                send.assert_not_called()
            self.assertEqual(row.w_sl.value(), 540)
            self.assertGreaterEqual(row.w_sl.maximum(), 540)
            self.assertEqual(row.w_sp.value(), 270)
            row.w_expand.click()
            row.w_sl.setValue(-720)
            self.assertAlmostEqual(math.degrees(self.ctrl.states[7].target), -360)
        finally:
            win.close()
            self.wait_until(lambda: win._allow_close)

    def _check_chinese_width_and_screenshot(self):
        self.ctrl.ids = pc.DEFAULT_IDS
        self.ctrl.states = {m: pc.MotorState(pc.spec_for(m)) for m in pc.DEFAULT_IDS}
        win = ui.Main(self.ctrl)
        win.show()
        win.resize(1280, 800)
        win._connection()
        self.wait_until(lambda: not win._busy())
        self.ctrl.refresh_offline()
        win._tick()
        self.app.processEvents()
        for row in win.rows.values():
            self.assertGreaterEqual(row.w_name.width(), row.w_name.fontMetrics().horizontalAdvance(row.w_name.text()))
        if os.environ.get('XIANDUCK_SCREENSHOT'):
            win.grab().save(os.environ['XIANDUCK_SCREENSHOT'])
        win.close()
        self.wait_until(lambda: not win.isVisible())

    def test_slow_connection_does_not_block_ui_and_close_waits(self):
        win = ui.Main(self.ctrl)
        win.show()
        real_connect = self.ctrl.connect
        def slow_connect():
            time.sleep(.2)
            return real_connect()
        with patch.object(self.ctrl, 'connect', side_effect=slow_connect):
            start = time.monotonic()
            win._connection()
            self.assertLess(time.monotonic() - start, .15)
            win.close()
            self.wait_until(lambda: not win.isVisible())
        self.assertFalse(self.ctrl.connected)
        self.assertIsNone(self.ctrl.thread)


if __name__ == '__main__':
    unittest.main()
