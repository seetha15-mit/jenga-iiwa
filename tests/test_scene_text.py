"""Checks the SDF files that jenga/scene.py writes, WITHOUT Drake.

Drake's modules are replaced by empty stand-ins for the duration of each test, so only our own text generation
runs.  This catches malformed files and wrong names; it cannot tell whether Drake accepts them (run_checks.py does)."""

import importlib
import sys
import tempfile
import types
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path
from unittest import mock

import numpy as np

from jenga import geometry as geo
from jenga.tower import TowerState
from tests.helpers import make_cfg

try:
    import yaml
except ImportError:  # pragma: no cover
    yaml = None

PREFIXES = ("jenga.scene", "jenga.controller", "jenga.kinematics", "utils.dtsystems", "utils.drake_models", "utils.viz")


def _fake_modules():
    class LeafSystem:
        def __init__(self, *a, **k):
            pass

        def __getattr__(self, name):
            return mock.MagicMock()

    class RigidTransform:
        pass

    def fake(name):
        m = types.ModuleType(name)
        m.__getattr__ = lambda attr: mock.MagicMock()
        return m

    allm = fake("pydrake.all")
    allm.LeafSystem, allm.RigidTransform = LeafSystem, RigidTransform
    pydrake = fake("pydrake")
    pydrake.all = allm
    return {
        "pydrake": pydrake, "pydrake.all": allm, "pydrake.geometry": fake("pydrake.geometry"),
        "manipulation": fake("manipulation"), "manipulation.station": fake("manipulation.station"),
    }


def parse_sdf(path):
    """The pset's SDF text uses drake: tags without declaring the namespace (Drake accepts it); add it to parse."""
    text = Path(path).read_text().replace("<sdf ", '<sdf xmlns:drake="http://drake.mit.edu" ', 1)
    return ET.fromstring(text)


class TestSceneFiles(unittest.TestCase):
    def build(self, **over):
        cfg = make_cfg(**over)
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        patcher = mock.patch.dict(sys.modules, _fake_modules())
        patcher.start()
        self.addCleanup(patcher.stop)
        for name in list(sys.modules):
            if name.startswith(PREFIXES):
                del sys.modules[name]
        scene = importlib.import_module("jenga.scene")
        tower = TowerState(cfg)
        sim = scene.Sim.__new__(scene.Sim)
        sim.cfg, sim.mobile, sim.asset_dir = cfg, cfg.BASE_MODE == "mobile", Path(tmp.name)
        sim.block_ids = sorted(tower.blocks)
        sim._friction = {b: 0.1 + 0.05 * b for b in sim.block_ids}
        return scene, sim, tower, cfg

    def test_assets_cover_table_platform_base_and_every_block(self):
        scene, sim, tower, cfg = self.build()
        a = sim._write_assets(tower)
        self.assertEqual(set(a["blocks"]), set(range(9)))
        for p in [a["table"], a["platform"], a["base"], *a["blocks"].values()]:
            self.assertTrue(Path(p).exists())
        self.assertEqual(parse_sdf(a["blocks"][3]).find("model").get("name"), "block_03")

    def test_fixed_base_writes_no_base_file(self):
        scene, sim, tower, cfg = self.build(BASE_MODE="fixed")
        self.assertIsNone(sim._write_assets(tower)["base"])

    def test_block_sdf_has_the_scaled_size_mass_and_its_own_hidden_friction(self):
        scene, sim, tower, cfg = self.build()
        a = sim._write_assets(tower)
        root = parse_sdf(a["blocks"][4])
        size = [float(v) for v in root.find(".//collision/geometry/box/size").text.split()]
        np.testing.assert_allclose(size, geo.block_dims(cfg), atol=1e-12)
        self.assertAlmostEqual(float(root.find(".//inertial/mass").text), geo.block_mass(cfg))
        self.assertAlmostEqual(float(root.find(".//{*}mu_dynamic").text), 0.1 + 0.05 * 4)
        self.assertIn("compliant_hydroelastic", Path(a["blocks"][4]).read_text())

    def test_point_contact_drops_the_hydroelastic_tags(self):
        scene, sim, tower, cfg = self.build(CONTACT_MODEL="point")
        a = sim._write_assets(tower)
        self.assertNotIn("hydroelastic", Path(a["blocks"][0]).read_text())

    def test_platform_height_and_footprint(self):
        scene, sim, tower, cfg = self.build()
        plat = parse_sdf(sim._write_assets(tower)["platform"])
        size = [float(v) for v in plat.find(".//collision/geometry/box/size").text.split()]
        self.assertAlmostEqual(size[2], cfg.PLATFORM_HEIGHT_M)
        self.assertAlmostEqual(size[0], geo.tower_footprint_m(cfg))  # no overhang for the fingers to catch

    def test_base_sdf_is_well_formed_with_the_right_joints(self):
        scene, sim, tower, cfg = self.build()
        root = parse_sdf(sim._write_assets(tower)["base"])
        joints = {j.get("name"): j for j in root.iter("joint")}
        # self.assertEqual({n: j.get("type") for n, j in joints.items()},
        #                  {"base_x": "prismatic", "base_y": "prismatic", "base_yaw": "revolute"})
        # self.assertEqual(joints["base_x"].find("parent").text, "world")
        # self.assertEqual(joints["base_yaw"].find("child").text, "base_link")
        self.assertEqual({n: j.get("type") for n, j in joints.items()}, {"base_yaw": "revolute"})
        self.assertEqual(joints["base_yaw"].find("parent").text, "world")
        self.assertEqual(joints["base_yaw"].find("child").text, "base_link")
        r = cfg.STATION_STANDOFF_M
        self.assertAlmostEqual(float(joints["base_yaw"].find("pose").text.split()[0]), r)  # axis at the tower centre
        
        self.assertIsNotNone(root.find(".//effort"))  # effort limits make the joints actuated
        self.assertIsNone(root.find(".//collision"))  # visual only: the base never collides

    def test_every_block_starts_exactly_where_the_tower_says(self):
        for bid in range(9):
            X = TowerState(make_cfg()).blocks[bid].X_WB
            q = geo.free_body_q(X)
            self.assertEqual(q.shape, (7,))
            self.assertAlmostEqual(float(np.linalg.norm(q[:4])), 1.0)
            np.testing.assert_allclose(q[4:], X[:3, 3])


class TestContactApproximation(unittest.TestCase):
    def setUp(self):
        patcher = mock.patch.dict(sys.modules, _fake_modules())
        patcher.start()
        self.addCleanup(patcher.stop)
        for name in list(sys.modules):
            if name.startswith(PREFIXES):
                del sys.modules[name]
        self.scene = importlib.import_module("jenga.scene")

    def test_only_the_chosen_name_is_looked_up(self):
        class Enum:  # this Drake has no kTamsi
            kSap, kLagged, kSimilar = "S", "L", "M"

        self.assertEqual(self.scene.contact_approximation(Enum, "sap"), "S")
        self.assertEqual(self.scene.contact_approximation(Enum, "similar"), "M")

    def test_a_missing_name_gives_a_helpful_error(self):
        class Enum:
            kSap, kLagged = "S", "L"

        with self.assertRaises(RuntimeError) as cm:
            self.scene.contact_approximation(Enum, "tamsi")
        self.assertIn("sap", str(cm.exception))
        self.assertIn("lagged", str(cm.exception))

    def test_api_hint_lists_matching_names(self):
        class Obj:
            set_controller_gains = 1
            other = 2

        self.assertIn("set_controller_gains", self.scene.api_hint(Obj, "gain"))
        self.assertEqual(self.scene.api_hint(Obj, "zzz"), "(nothing found)")


class FakePlant:
    """Just enough of MultibodyPlant for the actuator lookup (the pset's pattern from mobile_base_ik.py)."""

    class Joint:
        def __init__(self, name, instance):
            self._n, self._i = name, instance

        def name(self):
            return self._n

    class Actuator:
        def __init__(self, joint):
            self._j, self.gains = joint, None

        def joint(self):
            return self._j

        def set_controller_gains(self, gains):
            self.gains = gains

    def __init__(self, declared):
        self.joints = {}
        self.actuators = [self.Actuator(self.GetJointByName(n, i)) for n, i in declared]

    def GetJointByName(self, name, instance):
        return self.joints.setdefault((name, instance), self.Joint(name, instance))

    def GetJointActuatorIndices(self):
        return range(len(self.actuators))

    def get_joint_actuator(self, i):
        return self.actuators[i]


class TestActuatorLookup(unittest.TestCase):
    def setUp(self):
        patcher = mock.patch.dict(sys.modules, _fake_modules())
        patcher.start()
        self.addCleanup(patcher.stop)
        for name in list(sys.modules):
            if name.startswith(PREFIXES):
                del sys.modules[name]
        self.scene = importlib.import_module("jenga.scene")
        self.sim = self.scene.Sim.__new__(self.scene.Sim)
        self.sim.actuator_notes = []

    def test_sets_gains_on_the_right_joints_and_only_those(self):
        self.sim.plant = FakePlant([("j1", "arm"), ("j2", "arm"), ("j1", "other")])
        self.sim._set_gains("arm", ["j1", "j2"], [10.0, 20.0], [1.0, 2.0], "arm")
        a = self.sim.plant.actuators
        self.assertIsNotNone(a[0].gains)
        self.assertIsNotNone(a[1].gains)
        self.assertIsNone(a[2].gains)  # the same joint name on another instance is left alone
        self.assertIn("2 PD joints", self.sim.actuator_notes[0])

    def test_a_joint_without_an_actuator_is_an_error_that_lists_what_exists(self):
        self.sim.plant = FakePlant([("j1", "arm")])
        with self.assertRaises(RuntimeError) as cm:
            self.sim._set_gains("arm", ["j1", "j2"], [1.0, 1.0], [1.0, 1.0], "arm")
        self.assertIn("j2", str(cm.exception))
        self.assertIn("j1", str(cm.exception))

    def test_gain_count_mismatch_is_reported(self):
        self.sim.plant = FakePlant([])
        with self.assertRaises(RuntimeError):
            self.sim._set_gains("x", ["a", "b"], [1.0], [1.0], "arm")


class TestDrakeOrderingRules(unittest.TestCase):
    """Drake insists on an order: welds, physics settings and PD gains BEFORE Finalize(); default positions AFTER it
    (the first run of the hand-built scene failed on exactly this).  This reads the source, because Drake is not
    needed for the rule to be checked."""

    def setUp(self):
        text = Path("jenga/scene.py").read_text()
        start = text.index("    def _build_plant")
        end = text.index("    def _set_gains")
        self.body = text[start:end]

    def pos(self, needle):
        return self.body.index(needle)

    def test_finalize_comes_after_welds_physics_and_gains(self):
        fin = self.pos("plant.Finalize()")
        for needle in ("plant.WeldFrames", "set_gravity_enabled", "set_discrete_contact_approximation",
                       "self._set_gains(self.iiwa"):
            self.assertLess(self.pos(needle), fin, needle)

    def test_default_positions_come_after_finalize(self):
        fin = self.pos("plant.Finalize()")
        for needle in ("SetDefaultPositions(self.iiwa", "SetDefaultPositions(self.wsg", "SetDefaultPositions(inst"):
            self.assertGreater(self.pos(needle), fin, needle)


class TestQuaternion(unittest.TestCase):
    @staticmethod
    def to_matrix(q):
        w, x, y, z = q
        return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                         [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                         [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])

    def test_round_trip_on_random_and_special_rotations(self):
        rng = np.random.default_rng(1)

        def rot(axis, ang):
            a = axis / np.linalg.norm(axis)
            K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
            return np.eye(3) + np.sin(ang) * K + (1 - np.cos(ang)) * K @ K

        cases = [rot(rng.normal(size=3), rng.uniform(0, 2 * np.pi)) for _ in range(500)]
        cases += [rot(ax, ang) for ax in np.eye(3) for ang in (np.pi, np.pi / 2, 0.0)]
        for R in cases:
            np.testing.assert_allclose(self.to_matrix(geo.quat_wxyz(R)), R, atol=1e-9)


if __name__ == "__main__":
    unittest.main()
