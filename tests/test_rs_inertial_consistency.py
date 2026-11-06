"""Keep the legacy RS physics models aligned with the SDK RS URDF."""
import math
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
REFERENCE = ROOT / "urdf/RS/urdf/ReBot_Arm_RS.urdf"
LEGACY = ROOT / "urdf/00-arm-rs_asm-v3/urdf/00-arm-rs_asm-v3.urdf"
MJCF = ROOT / "mjcf/rebot_devarm/rebot_devarm.xml"
LINKS = ("base_link", "link1", "link2", "link3", "link4", "link5",
         "link6", "gripper_end", "gripper_left", "gripper_right")


def numbers(value):
    return [float(x) for x in value.split()]


class TestRSInertialConsistency(unittest.TestCase):
    def assert_values(self, expected, actual, tolerance):
        self.assertEqual(len(expected), len(actual))
        for a, b in zip(expected, actual):
            self.assertTrue(math.isclose(a, b, rel_tol=0.0, abs_tol=tolerance),
                            f"{a} != {b}")

    def test_legacy_urdf_and_mjcf(self):
        source = ET.parse(REFERENCE).getroot()
        legacy = ET.parse(LEGACY).getroot()
        mjcf = ET.parse(MJCF).getroot()
        for name in LINKS:
            with self.subTest(link=name):
                a = source.find(f"./link[@name='{name}']/inertial")
                b = legacy.find(f"./link[@name='{name}']/inertial")
                c = mjcf.find(f".//body[@name='{name}']/inertial")
                self.assertIsNotNone(a)
                self.assertIsNotNone(b)
                self.assertIsNotNone(c)
                self.assert_values([float(a.find("mass").get("value"))],
                                   [float(b.find("mass").get("value"))], 1e-10)
                self.assert_values([float(a.find("mass").get("value"))],
                                   [float(c.get("mass"))], 1e-10)
                apos = numbers(a.find("origin").get("xyz"))
                self.assert_values(apos, numbers(b.find("origin").get("xyz")), 1e-10)
                self.assert_values(apos, numbers(c.get("pos")), 1e-10)
                keys = ("ixx", "iyy", "izz", "ixy", "ixz", "iyz")
                ai = [float(a.find("inertia").get(k)) for k in keys]
                bi = [float(b.find("inertia").get(k)) for k in keys]
                self.assert_values(ai, bi, 1e-12)
                self.assert_values(ai, numbers(c.get("fullinertia")), 1e-12)


if __name__ == "__main__":
    unittest.main()
