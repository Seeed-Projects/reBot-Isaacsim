"""Offline pose regressions; mutations stay in anonymous USD session layers."""

import math
import unittest

import numpy as np
from pxr import Gf, UsdGeom, UsdPhysics

import validate_physics_fidelity as fidelity


class UsdPoseTests(unittest.TestCase):
    def setUp(self):
        failures = []
        _, _, self.frames = fidelity.parse_urdf(fidelity.URDF_PATH, failures)
        self.assertEqual(failures, [])

    def open_stage(self, variant):
        stage = fidelity.open_variant(fidelity.USD_PATH, variant)
        links = {
            prim.GetName(): prim for prim in stage.Traverse()
            if UsdPhysics.MassAPI(prim)
            and UsdPhysics.MassAPI(prim).GetMassAttr().HasAuthoredValue()
        }
        return stage, links

    def check_pose(self, stage, links):
        failures, metrics = [], {}
        fidelity.check_usd_pose(stage, self.frames, links, failures, metrics, "test")
        return failures

    def test_current_variants_preserve_zero_pose_and_gripper_axes(self):
        self.assertEqual(self.frames["gripper_end"]["joint"], "j_gripper_end")
        self.assertIn("gripper_left", self.frames)
        self.assertIn("gripper_right", self.frames)
        for variant in ("physics", "mujoco"):
            with self.subTest(variant=variant):
                stage, links = self.open_stage(variant)
                self.assertEqual(self.check_pose(stage, links), [])
                cache = UsdGeom.XformCache()
                base = np.asarray(cache.GetLocalToWorldTransform(links["base_link"])).T
                gripper = np.asarray(cache.GetLocalToWorldTransform(links["gripper_end"])).T
                relative = np.linalg.inv(base) @ gripper
                np.testing.assert_allclose(relative[:3, 0], [1, 0, 0], atol=1e-4)
                np.testing.assert_allclose(relative[:3, 1], [0, 1, 0], atol=1e-4)

    def test_quarter_turn_of_gripper_body_is_rejected(self):
        stage, links = self.open_stage("physics")
        orient = links["gripper_end"].GetAttribute("xformOp:orient")
        turn = Gf.Quatf(math.sqrt(0.5), Gf.Vec3f(math.sqrt(0.5), 0, 0))
        orient.Set(orient.Get() * turn)
        failures = self.check_pose(stage, links)
        self.assertTrue(any("j_gripper_end: USD body zero-pose frame" in f for f in failures))
        self.assertFalse(any("USD joint zero-pose frame" in f for f in failures))

    def test_quarter_turn_of_fixed_joint_is_rejected(self):
        stage, links = self.open_stage("physics")
        prim = next(p for p in stage.Traverse() if p.GetName() == "j_gripper_end")
        orient = UsdPhysics.Joint(prim).GetLocalRot0Attr()
        turn = Gf.Quatf(math.sqrt(0.5), Gf.Vec3f(math.sqrt(0.5), 0, 0))
        orient.Set(orient.Get() * turn)
        failures = self.check_pose(stage, links)
        self.assertTrue(any("j_gripper_end: USD joint zero-pose frame" in f for f in failures))
        self.assertFalse(any("USD body zero-pose frame" in f for f in failures))

    def test_non_finite_body_and_joint_frames_are_rejected(self):
        for source in ("body", "joint"):
            with self.subTest(source=source):
                stage, links = self.open_stage("physics")
                if source == "body":
                    position = links["gripper_end"].GetAttribute("xformOp:translate")
                    position.Set(Gf.Vec3d(float("nan"), 0, 0))
                else:
                    prim = next(p for p in stage.Traverse() if p.GetName() == "j_gripper_end")
                    position = UsdPhysics.Joint(prim).GetLocalPos0Attr()
                    position.Set(Gf.Vec3f(float("nan"), 0, 0))
                failures = self.check_pose(stage, links)
                message = f"j_gripper_end: USD {source} zero-pose frame is non-finite"
                self.assertTrue(any(message in failure for failure in failures))


if __name__ == "__main__":
    unittest.main()
