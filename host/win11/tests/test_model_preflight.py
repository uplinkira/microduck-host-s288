"""只读静态报告不认证物理训练；测试使用临时 MJCF。"""
from pathlib import Path
import sys
import tempfile
import unittest
import hashlib
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from model_preflight import inspect_model
import model_preflight as mp
from robot_profile import profile_digest
from test_robot_profile import complete


class PreflightTests(unittest.TestCase):
    def test_assets_hash_and_package_escape(self):
        with tempfile.TemporaryDirectory() as folder:
            package = Path(folder) / 'package'
            package.mkdir()
            asset = package / 'a.stl'
            asset.write_bytes(b'asset bytes')
            path = package / 'robot.xml'
            path.write_text('<mujoco><asset><mesh file="a.stl"/><mesh file="../private.stl"/><texture file="https://example.com/a"/></asset></mujoco>', encoding='utf-8')
            r = inspect_model(path)
            self.assertEqual(r['assets'], [{'path': str(asset.resolve()), 'sha256': hashlib.sha256(b'asset bytes').hexdigest()}])
            self.assertIn('resource_outside_package', [c['code'] for c in r['checks']])
            self.assertIn('resource_uri_or_absolute', [c['code'] for c in r['checks']])
            with patch.object(mp, 'MAX_ASSET_BYTES', 3):
                r = inspect_model(path)
            self.assertEqual(r['assets'], [])
            self.assertIn('asset_size_limit', [c['code'] for c in r['checks']])

    def test_xml_resource_limits_encoding_and_early_profile_digest(self):
        p = complete()
        r = self.inspect('<?xml version="1.0" encoding="UTF-16"?><mujoco/>', p)
        self.assertEqual(r['profile_digest'], profile_digest(p))
        self.assertIn('xml_encoding', [c['code'] for c in r['checks']])
        with patch.object(mp, 'MAX_XML_BYTES', 5):
            r = self.inspect('<mujoco/>', p)
        self.assertIn('xml_size_limit', [c['code'] for c in r['checks']])
        with patch.object(mp, 'MAX_XML_NODES', 2):
            r = self.inspect('<mujoco><worldbody><body/></worldbody></mujoco>')
        self.assertFalse(r['ok'])

    def test_unknown_model_generators_numeric_fields_and_mass_warning(self):
        r = self.inspect('<mujoco><compiler balanceinertia="true" boundinertia="0.1"/><worldbody><body><freejoint/><inertial mass="103" diaginertia="1 1 1"/><joint name="head" axis="nan 0 1" range="0 inf"/><replicate count="2"><geom scale="1 bad 1"/></replicate><attach model="other"/></body></worldbody></mujoco>')
        codes = [c['code'] for c in r['checks']]
        for code in ('attach_unresolved', 'replicate_unresolved', 'nonfinite_axis', 'nonfinite_range', 'nonfinite_scale', 'mass_magnitude', 'compiler_balanceinertia', 'compiler_boundinertia'):
            self.assertIn(code, codes)
        self.assertFalse(r['ok'])

    def inspect(self, xml, profile=None):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'robot.xml'
            path.write_text(xml, encoding='utf-8')
            before = path.read_bytes()
            result = inspect_model(path, profile)
            self.assertEqual(path.read_bytes(), before)
            self.assertEqual(len(list(Path(folder).iterdir())), 1)
            return result

    def test_static_success_is_not_training_certification(self):
        r = self.inspect('<mujoco><worldbody><body><freejoint/><inertial mass="1" diaginertia="1 1 1"/><body><joint name="head"/><inertial mass="1" diaginertia="1 1 1"/></body></body></worldbody></mujoco>', complete())
        self.assertTrue(r['ok'])
        self.assertFalse(r['certified'])
        self.assertFalse(r['training_ready'])
        self.assertEqual(r['status'], 'review_required')
        self.assertEqual(r['explicit_mass_sum'], 2)

    def test_missing_include_and_defaults_are_not_resolved(self):
        r = self.inspect('<mujoco><include file="missing.xml"/><default><joint damping="1"/></default></mujoco>', complete())
        codes = [c['code'] for c in r['checks']]
        self.assertIn('include_unresolved', codes)
        self.assertIn('default_unresolved', codes)
        self.assertIn('profile_joint_missing', codes)
        self.assertFalse(r['certified'])

    def test_bad_mass_inertia_and_fixed_base(self):
        r = self.inspect('<mujoco><worldbody><body name="bad"><inertial mass="-1" diaginertia="1 1 4"/><joint name="head"/></body></worldbody></mujoco>')
        codes = [c['code'] for c in r['checks']]
        self.assertFalse(r['ok'])
        for code in ('mass_invalid', 'inertia_invalid', 'floating_base_missing'):
            self.assertIn(code, codes)

    def test_mesh_and_profile_missing(self):
        r = self.inspect('<mujoco><asset><mesh file="missing.stl"/></asset><worldbody><body><freejoint/><joint name="other"/></body></worldbody></mujoco>', complete())
        codes = [c['code'] for c in r['checks']]
        self.assertIn('mesh_missing', codes)
        self.assertIn('profile_joint_missing', codes)

    def test_malformed_and_entity_declarations_fail_closed(self):
        for xml in ('<mujoco>', '<robot/>', '<!DOCTYPE mujoco [<!ENTITY x "abc">]><mujoco/>'):
            r = self.inspect(xml)
            self.assertFalse(r['ok'])


if __name__ == '__main__':
    unittest.main()
