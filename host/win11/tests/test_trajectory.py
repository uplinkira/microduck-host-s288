"""离线规划与导出契约测试；使用真实 Ruckig，不接串口。"""
import copy
import csv
import json
import math
from pathlib import Path
import sys
import tempfile
import threading
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import trajectory as tr
from trajectory_export import export_reference
from robot_profile import make_profile


def pose(angles, speed=12.5, dwell=0, name='动作'):
    return dict(name=name, angles=angles, speed=speed, dwell=dwell)


def profile():
    return make_profile([dict(motor_id=mid, semantic=f'joint_{mid}', model_joint=f'模型关节_{mid}',
        bus='COM4', direction=direction, q_home_rad=home, role='policy', policy_index=i,
        verified=True) for i, (mid, direction, home) in enumerate([(1, -1, .2), (7, 1, -.3)])])


class TrajectoryTests(unittest.TestCase):
    def test_stop_boundaries_pause_and_derivative_limits(self):
        poses = [pose({7: 10, 1: -2}, dwell=.123), pose({7: -20, 1: 4}, speed=9, dwell=.08)]
        plan = tr.plan_poses(poses, start_angles={1: 0, 7: 0}, sample_hz=200)
        self.assertEqual(plan['joint_ids'], [1, 7])
        self.assertFalse(plan['backend']['cloud'])
        self.assertEqual(plan['start_source'], 'explicit_angles')
        samples = plan['samples']
        self.assertTrue(all(a['t_s'] < b['t_s'] for a, b in zip(samples, samples[1:])))
        self.assertEqual(samples[-1]['t_s'], plan['duration_s'])
        for p, segment in zip(poses, plan['segments']):
            end = next(s for s in samples if s['t_s'] == segment['motion_end_s'])
            self.assertEqual(end['q_deg'], [p['angles'][i] for i in plan['joint_ids']])
            self.assertEqual(end['dq_deg_s'], [0, 0])
            self.assertEqual(end['ddq_deg_s2'], [0, 0])
            self.assertAlmostEqual(segment['end_s'] - segment['motion_end_s'], p['dwell'])
            part = [s for s in samples if segment['start_s'] <= s['t_s'] <= segment['end_s']]
            for s in part:
                self.assertLessEqual(max(map(abs, s['dq_deg_s'])), p['speed'] + 1e-7)
                self.assertLessEqual(max(map(abs, s['ddq_deg_s2'])), 25 + 1e-7)
                if s['t_s'] >= segment['motion_end_s']:
                    self.assertEqual(s['q_deg'], end['q_deg'])
            for a, b in zip(part, part[1:]):
                dt = b['t_s'] - a['t_s']
                j = max(abs(x-y)/dt for x, y in zip(a['ddq_deg_s2'], b['ddq_deg_s2']))
                self.assertLessEqual(j, 100 + 1e-6)
        json.dumps(plan, allow_nan=False)

    def test_first_pose_defines_start_and_preserves_pause(self):
        plan = tr.plan_poses([pose({1: 100, 7: -50}, dwell=.12), pose({1: 110, 7: -48})])
        self.assertEqual(plan['samples'][0]['q_deg'], [100, -50])
        self.assertEqual(plan['segments'][0]['motion_end_s'], 0)
        self.assertEqual(plan['segments'][1]['start_s'], .12)
        self.assertEqual(plan['start_source'], 'first_pose')

    def test_zero_motion_no_duplicate_times(self):
        plan = tr.plan_poses([pose({1: 1}), pose({1: 1}), pose({1: 1}, dwell=.031)])
        self.assertEqual([s['t_s'] for s in plan['samples']], [0, .02, .031])
        self.assertEqual(tr.plan_poses([pose({1: 1})])['duration_s'], 0)

    def test_missing_ids_nonfinite_cloud_and_resource_inputs(self):
        invalid = [[], [pose({1: 0}), pose({2: 0})], [pose({15: 0})],
                   [pose({1: float('nan')})], [pose({1: 0}, speed=0)],
                   [pose({1: 0}, dwell=-1)], [dict(pose({1: 0}), mode='blend')],
                   [dict(pose({1: 0}), intermediate_positions=[])],
                   [pose({1: 0, '1': 2})], [pose({1: 0}, speed=True)]]
        for poses in invalid:
            with self.subTest(poses=poses), self.assertRaises(ValueError):
                tr.plan_poses(poses)
        for params in [dict(sample_hz=0), dict(max_jerk=float('inf')),
                       dict(max_acceleration=-1), dict(start_angles={2: 1})]:
            with self.subTest(params=params), self.assertRaises(ValueError):
                tr.plan_poses([pose({1: 0})], **params)
        with patch.object(tr, 'MAX_SAMPLES', 10), self.assertRaises(ValueError):
            tr.plan_poses([pose({1: 0}, dwell=1)])
        with self.assertRaises(ValueError):
            tr.plan_poses([pose({1: 0}), pose({1: 100000}, speed=.001)])

    def test_cancel_before_and_during_sampling(self):
        event = threading.Event()
        event.set()
        with self.assertRaises(tr.PlanningCancelled):
            tr.plan_poses([pose({1: 0})], cancel=event)
        calls = 0
        def cancel():
            nonlocal calls
            calls += 1
            return calls > 20
        with self.assertRaises(tr.PlanningCancelled):
            tr.plan_poses([pose({1: 0}), pose({1: 100})], cancel=cancel)

    def test_wrong_backend_does_not_fallback(self):
        import ruckig
        with patch.object(ruckig, '__version__', '0.0'), self.assertRaises(RuntimeError):
            tr.plan_poses([pose({1: 0})])


class ExportTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        self.plan = tr.plan_poses([pose({1: 10, 7: 20}), pose({1: 12, 7: 25}, dwell=.05)])

    def tearDown(self):
        self.temp.cleanup()

    def test_host_draft_and_model_transform_do_not_mutate_plan(self):
        original = copy.deepcopy(self.plan)
        draft = profile()
        draft['joints'][0].update(direction=None, q_home_rad=None, verified=False)
        export_reference(self.root/'draft.json', self.plan, draft)
        raw = json.loads((self.root/'draft.json').read_text(encoding='utf-8'))
        self.assertFalse(raw['training_ready'])
        self.assertTrue(raw['profile_checks'])
        export_reference(self.root/'model.json', self.plan, profile(), True)
        model = json.loads((self.root/'model.json').read_text(encoding='utf-8'))
        self.assertEqual(model['coordinate_frame'], 'model_joint_coordinates')
        self.assertAlmostEqual(model['samples'][0]['q_rad'][0], .2-math.radians(10))
        self.assertAlmostEqual(model['samples'][0]['q_rad'][1], -.3+math.radians(20))
        for old, new in zip(self.plan['samples'], model['samples']):
            self.assertAlmostEqual(new['dq_rad_s'][0], -math.radians(old['dq_deg_s'][0]))
        self.assertEqual(self.plan, original)
        self.assertFalse(model['training_ready'])
        self.assertTrue(model['missing_training_fields'])

    def test_incomplete_multibus_and_unknown_mapping_rejected(self):
        variants = [None]
        for field, value in [('direction', None), ('q_home_rad', None), ('verified', False), ('bus', 'COM5')]:
            p = profile(); p['joints'][0][field] = value; variants.append(p)
        p = profile(); p['joints'][0]['motor_id'] = 2; variants.append(p)
        for p in variants:
            with self.subTest(profile=p), self.assertRaises(ValueError):
                export_reference(self.root/'bad.json', self.plan, p, True)
        self.assertFalse((self.root/'bad.json').exists())

    def test_csv_chinese_header_timestamp_and_metadata(self):
        export_reference(self.root/'模型.csv', self.plan, profile(), True)
        with (self.root/'模型.csv').open(encoding='utf-8-sig', newline='') as stream:
            rows = list(csv.DictReader(stream))
        self.assertIn('模型关节_1__q_rad', rows[0])
        self.assertEqual(rows[0]['training_ready'], 'false')
        self.assertEqual(float(rows[-1]['t_s']), self.plan['duration_s'])
        self.assertEqual(len(rows), len(self.plan['samples']))

    def test_export_rejects_corruption_before_overwrite(self):
        target = self.root/'existing.json'
        export_reference(target, self.plan)
        before = target.read_bytes()
        for mutate in [lambda p: p['samples'][1].update(t_s=0),
                       lambda p: p['samples'][0].update(q_deg=[0]),
                       lambda p: p['units'].update(position='rad'),
                       lambda p: p['samples'][0]['q_deg'].__setitem__(0,float('nan'))]:
            corrupt = copy.deepcopy(self.plan); mutate(corrupt)
            with self.assertRaises(ValueError):
                export_reference(target, corrupt)
            self.assertEqual(target.read_bytes(), before)
        with self.assertRaises(ValueError):
            export_reference(self.root/'bad.txt', self.plan)


if __name__ == '__main__':
    unittest.main()
