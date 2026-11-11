"""假总线/单调时钟测试，不连接实机、不使用假数据冒充实测。"""
import csv
import json
import math
import os
from pathlib import Path
import sys
import tempfile
import threading
import time
import unittest
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
os.environ.setdefault('QT_QPA_PLATFORM','offscreen')
from measurement import MeasurementRunner,MeasurementConfig,TRAJECTORIES,trajectory
import pose_core as pc
import pose_ui as ui
from PyQt6.QtWidgets import QApplication


class Clock:
    def __init__(self):self.t=0.0
    def __call__(self):return self.t
    def wait(self,dt):self.t+=dt


class FakeBus:
    def __init__(self,clock,center=0):
        self.clock=clock; self.calls=[]; self.center=center; self.fault=None; self.closed=False
    def transact(self,*args,**kwargs):
        self.calls.append((args,kwargs)); self.clock.t+=.001
        if self.fault:return self.fault(args,kwargs)
        return dict(motor_id=args[0],outputPos=args[4] if args[5] else self.center,outputSpd=0.,
                    outputTor=.05,vol=11.8,Temp=30.,MError=0,timeout=0)
    def release(self,*args,**kwargs):pass
    def probe(self,mid):return self.transact(mid,1,0,0,self.center,0,0)
    def close(self):self.closed=True


class MeasurementTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory(); self.clock=Clock(); self.bus=FakeBus(self.clock,center=1.2)
        self.stop=threading.Event()
        self.runner=MeasurementRunner(self.bus,threading.RLock(),self.tmp.name,self.stop,clock=self.clock,wait=self.clock.wait)
    def tearDown(self):self.tmp.cleanup()
    def active(self,**kw):
        return MeasurementConfig(kind='position',ids=(7,),fixture_confirmed=True,operator='测试',firmware='test-only',**kw)
    def test_trajectory_math_and_drop(self):
        self.assertEqual(trajectory('lift_and_drop',2),(-math.pi/2,False))
        self.assertAlmostEqual(trajectory('up_and_down',3)[0],math.pi/2)
        self.assertAlmostEqual(trajectory('up_and_down',6)[0],.8*math.pi/2)
        self.assertAlmostEqual(trajectory('sin_time_square',1)[0],math.sin(1))
    def test_validation_rejects_unconfirmed_physics_and_protocol(self):
        for config in [self.active(gains=(float('nan'),)),self.active(gains=(9999,)),
                       MeasurementConfig(kind='trajectory'),MeasurementConfig(ids=(15,)),MeasurementConfig(ids=(7,7)),
                       MeasurementConfig(kind='position',ids=(1,2))]:
            with self.assertRaises(ValueError):self.runner.run(config,centers={7:1.2})
        self.assertFalse(self.bus.calls)
    def test_latency_1000_rounds_and_csv_schema(self):
        folder,data=self.runner.run(MeasurementConfig(ids=tuple(range(1,14)),cycles=1000))
        self.assertEqual(data['status'],'completed')
        self.assertEqual(data['summary']['samples'],13000)
        self.assertEqual(data['summary']['complete_rounds'],1000)
        self.assertAlmostEqual(data['summary']['round_ms']['mean'],13,places=5)
        self.assertTrue(all(args[2:4]==(0.,0.) and args[5:7]==(0,0) for args,_ in self.bus.calls))
        with (folder/'samples.csv').open(encoding='utf-8-sig') as stream:
            rows=list(csv.DictReader(stream))
        self.assertEqual(len(rows),13000+13+39)
        self.assertEqual(json.loads((folder/'session.json').read_text(encoding='utf-8'))['summary']['valid'],13000)
    def test_position_uses_absolute_center_zero_feedforward_and_deadlines(self):
        folder,result=self.runner.run(self.active(duration_s=1,hz=50),centers={7:1.2})
        self.assertEqual(result['status'],'completed')
        rows=json.loads((folder/'trial_01.json').read_text(encoding='utf-8'))['entries']
        self.assertTrue(any(abs(r['q_cmd_abs_rad']-(1.2+math.radians(3)))<1e-7 for r in rows))
        self.assertTrue(all(r['dq_cmd_rad_s']==0 and r['tau_ff_Nm']==0 for r in rows))
        self.assertAlmostEqual(result['summary']['effective_hz'],50,places=5)
        self.assertAlmostEqual(result['summary']['tracking_rmse_rad'],0,places=5)
        self.assertEqual([c[0][5:7] for c in self.bus.calls[-3:]],[(0,0)]*3)
    def test_nine_pendulum_trials_and_drop_zero_gain(self):
        cfg=MeasurementConfig(kind='trajectory',ids=(7,),trajectories=TRAJECTORIES,gains=(3.,5.,10.),
                              mass=.1,arm_mass=.02,length=.1,pendulum_direction=-1,
                              operator='测试',firmware='test',fixture_confirmed=True,fixture='均匀杆向下零位')
        folder,result=self.runner.run(cfg,centers={7:1.2})
        self.assertEqual(result['status'],'completed',result['errors'])
        self.assertEqual(len(list(folder.glob('trial_*.json'))),9)
        data=json.loads((folder/'trial_03.json').read_text(encoding='utf-8'))
        released=[r for r in data['entries'] if not r['torque_enable']]
        self.assertTrue(released)
        self.assertTrue(all(r['kp_Nm_rad']==r['kd_Nm_s_rad']==0 for r in released))
        self.assertFalse(data['bam_compatible'])
        self.assertEqual(data['arm_mass'],.02)
    def test_wrong_id_and_no_feedback_not_success(self):
        self.bus.fault=lambda a,k:dict(motor_id=14,outputPos=0,outputSpd=0,outputTor=0,vol=12,Temp=30,MError=0)
        folder,data=self.runner.run(MeasurementConfig(cycles=3))
        self.assertEqual(data['summary']['valid'],0)
        self.assertEqual(data['summary']['missing'],3)
        self.assertEqual(data['status'],'failed')
        self.assertIsNone(data['summary']['roundtrip_ms']['mean'])
    def test_fault_and_partial_capture_still_release(self):
        original=self.bus.transact
        def faulty(*args,**kwargs):
            fb=original(*args,**kwargs)
            if len(self.bus.calls)>3:fb['Temp']=80
            return fb
        self.bus.transact=faulty
        folder,data=self.runner.run(self.active(),centers={7:1.2})
        self.assertEqual(data['status'],'failed')
        self.assertTrue((folder/'trial_01.json').exists())
        self.assertEqual([a[5:7] for a,k in self.bus.calls[-3:]],[(0,0)]*3)
    def test_cancel_does_not_return_to_zero(self):
        def wait(dt):self.clock.wait(dt); self.stop.set()
        self.runner.wait=wait
        folder,data=self.runner.run(self.active(),centers={7:1.2})
        self.assertEqual(data['status'],'cancelled')
        self.assertFalse(any(r['stage']=='return' for r in self.runner.rows))
        self.assertEqual(len([r for r in self.runner.rows if r['stage']=='release']),3)
    def test_write_failure_before_session_sends_no_commands(self):
        with patch('measurement.Path.write_text',side_effect=OSError('disk full')):
            with self.assertRaises(OSError):self.runner.run(MeasurementConfig())
        self.assertFalse(self.bus.calls)

    def test_tracking_error_debounce_and_immediate_large_error(self):
        self.runner.config=self.active(); self.runner.large_error_since=None
        row=dict(feedback_valid=True,merror=0,temp=30,input_volts=11.8,timeout=0,
                 torque_feedback_Nm=.05,torque_enable=True,position=math.radians(20),goal_position=0)
        self.runner._check(row,True)
        self.clock.t+=.1; self.runner._check(row,True)
        self.clock.t+=.11
        with self.assertRaises(RuntimeError):self.runner._check(row,True)
        row['position']=0; self.runner._check(row,True)
        self.assertIsNone(self.runner.large_error_since)
        row['position']=math.radians(46)
        with self.assertRaises(RuntimeError):self.runner._check(row,True)

    def test_active_dropped_feedback_aborts_and_releases(self):
        original=self.bus.transact
        def dropped(*args,**kwargs):
            fb=original(*args,**kwargs)
            return None if len(self.bus.calls)==4 else fb
        self.bus.transact=dropped
        folder,data=self.runner.run(self.active(),centers={7:1.2})
        self.assertEqual(data['status'],'failed')
        self.assertEqual(data['summary']['missing'],1)
        self.assertTrue(all(data['release_feedback'].values()))
        self.assertFalse(any(r['stage']=='return' for r in self.runner.rows))


class IntegrationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):cls.app=QApplication.instance() or QApplication([])
    def test_reservation_blocks_normal_controls_and_estop_cancels(self):
        ctrl=pc.PoseController(None,6000000,False,[7]); ctrl.connected=True; ctrl.bus=FakeBus(Clock())
        ctrl.states[7].centered=True; ctrl.states[7].fb={'outputPos':0}
        cfg=MeasurementConfig(cycles=2)
        ctrl.reserve_measurement(cfg)
        self.assertFalse(ctrl.set_enabled(7,True)); self.assertFalse(ctrl.set_target(7,5))
        ctrl.goto({7:5}); self.assertFalse(ctrl.states[7].enabled)
        self.assertFalse(ctrl.start_range()); self.assertIsNone(ctrl._probe(7))
        ctrl.release_all(); self.assertTrue(ctrl.measurement_stop.is_set())
        with tempfile.TemporaryDirectory() as d:
            result=ctrl.collect_measurement(cfg,d)
        self.assertIn('cancelled',result); self.assertFalse(ctrl.measurement_active)
        self.assertFalse(ctrl.states[7].enabled); self.assertTrue(ctrl.thread.is_alive())
        ctrl.shutdown(); self.assertFalse(ctrl.connected)
    def test_ui_immutable_config_and_no_fake_measurement(self):
        ctrl=pc.PoseController(None,6000000,True,pc.DEFAULT_IDS); ctrl.connected=True
        win=ui.Main(ctrl); dialog=ui.MeasurementDialog(win,ctrl)
        self.assertEqual(dialog._config().cycles,1000)
        dialog.kind.setCurrentIndex(1); self.assertEqual(len(dialog._config().ids),13)
        dialog._start(); self.assertIn('真实串口',dialog.out.text()); self.assertFalse(ctrl.measurement_active)
        dialog.close(); win._allow_close=True; win.close(); ctrl.connected=False
    def test_export_calibration_preserves_unconfirmed_direction(self):
        ctrl=pc.PoseController(None,6000000,True,[7]); ctrl.calib={7:{'ex_zero':1.2}}
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'calib.json'; ctrl.export_calibration(p)
            data=json.loads(p.read_text(encoding='utf-8')); self.assertEqual(data['signs'],{})
            self.assertEqual(data['zeros']['7']['ex_zero'],1.2)
        from mapping_ui import validate_rows
        self.assertEqual(validate_rows([{'direction':'','q_home_rad':''}])[0]['direction'],'')
        with self.assertRaises(ValueError):validate_rows([{'direction':'2','q_home_rad':''}])

    def test_full_protocol_tuning_and_ui_slider_endpoints(self):
        ctrl=pc.PoseController(None,6000000,True,[7]); win=ui.Main(ctrl)
        win.kp.setValue(10000); win.kd.setValue(10000); win.speed.setValue(10000)
        self.assertAlmostEqual(ctrl.kp,pc.KP_MAX)
        self.assertAlmostEqual(ctrl.kd,pc.KD_MAX)
        self.assertAlmostEqual(ctrl.speed,pc.SPEED_MAX)
        win.kp.setValue(0); win.kd.setValue(0); win.speed.setValue(0)
        self.assertEqual((ctrl.kp,ctrl.kd,ctrl.speed),(0,0,0))
        win.kp_spin.setValue(3.25); win.kd_spin.setValue(.125); win.speed_spin.setValue(12.5)
        self.assertEqual((ctrl.kp,ctrl.kd,ctrl.speed),(3.25,.125,12.5))
        with self.assertRaises(ValueError):ctrl.set_gain_values(pc.KP_MAX+1,0)
        with self.assertRaises(ValueError):ctrl.set_speed(pc.SPEED_MAX+1)
        self.assertEqual(ctrl.transition({7:5},1,threading.Event()),0)
        win._allow_close=True; win.close()

if __name__=='__main__':unittest.main()
