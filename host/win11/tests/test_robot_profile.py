"""构型缺失值、地址语义、变更失效与文件持久化回归。"""
import copy
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import robot_profile as rp


def complete():
    return rp.make_profile([dict(motor_id=7, semantic='head_pitch', model_joint='head',
                                bus='COM3', direction=1, q_home_rad=0.2,
                                role='policy', policy_index=0, verified=True)])


class ProfileTests(unittest.TestCase):
    def test_name_joint_count_and_json_resource_limits(self):
        p = complete()
        p['name'] = ' '
        with self.assertRaises(ValueError):
            rp.validate_profile(p)
        p['name'] = 'x' * 129
        with self.assertRaises(ValueError):
            rp.validate_profile(p)
        p = complete()
        p['joints'] *= 129
        with self.assertRaises(ValueError):
            rp.validate_profile(p)
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'p.json'
            rp.write_json(path, complete())
            before = path.read_bytes()
            with patch.object(rp, 'MAX_JSON_BYTES', 10):
                with self.assertRaises(ValueError):
                    rp.read_json(path)
                with self.assertRaises(ValueError):
                    rp.write_json(path, complete())
            self.assertEqual(path.read_bytes(), before)

    def test_draft_never_invents_calibration_or_model_name(self):
        p = rp.draft_from_hardware({7: {'joint': 'head_pitch', 'verified': True}})
        for key in ('direction', 'q_home_rad', 'bus', 'model_joint', 'policy_index'):
            self.assertIsNone(p['joints'][0][key])
        self.assertFalse(p['joints'][0]['verified'])
        self.assertIn('missing_direction', [c['code'] for c in rp.validate_profile(p)])

    def test_addresses_and_broadcast(self):
        p = complete()
        other = dict(p['joints'][0], semantic='other', model_joint='other', policy_index=1)
        p['joints'].append(other)
        with self.assertRaises(ValueError):
            rp.validate_profile(p)
        other['bus'] = 'COM4'
        self.assertEqual(rp.validate_profile(p), [])
        other['motor_id'] = 15
        with self.assertRaises(ValueError):
            rp.validate_profile(p)

    def test_invalid_values_and_indices(self):
        for key, value in [('direction', True), ('q_home_rad', float('nan')),
                           ('policy_index', 2), ('verified', 'yes')]:
            p = complete()
            p['joints'][0][key] = value
            with self.assertRaises(ValueError, msg=key):
                rp.validate_profile(p)

    def test_digest_order_independent_and_change_invalidates(self):
        p = complete()
        q = dict(reversed(list(p.items())))
        self.assertEqual(rp.profile_digest(p), rp.profile_digest(q))
        self.assertFalse(rp.diff_profiles(p, q)['validation_invalidated'])
        q = copy.deepcopy(p)
        q['joints'][0]['direction'] = -1
        diff = rp.diff_profiles(p, q)
        self.assertTrue(diff['validation_invalidated'])
        self.assertTrue(diff['changes'])
        self.assertEqual(p['joints'][0]['direction'], 1)

    def test_json_roundtrip_and_failed_write_preserves_old_file(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / '构型.json'
            p = complete()
            p['name'] = '小鸭'
            rp.write_json(path, p)
            self.assertEqual(rp.read_json(path), p)
            with self.assertRaises(ValueError):
                rp.write_json(path, {'bad': float('inf')})
            self.assertEqual(rp.read_json(path), p)
            self.assertIn('小鸭', path.read_text(encoding='utf-8'))


if __name__ == '__main__':
    unittest.main()
