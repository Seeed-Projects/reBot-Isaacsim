#!/usr/bin/env python3
"""Regression tests for dynamic evidence and VALIDATION.md generation."""

from copy import deepcopy
import json
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import textwrap
import unittest
from unittest import mock

SCRIPT_DIR = Path(__file__).resolve().parent
PACKAGE_DIR = SCRIPT_DIR.parent
sys.path.insert(0, str(SCRIPT_DIR))

from dynamic_evidence_contract import (  # noqa: E402
    atomic_write_json,
    dynamic_evidence_problems,
)


class ValidationEvidenceTests(unittest.TestCase):
    def load_report(self, engine):
        path = PACKAGE_DIR / f"evidence/physics_fidelity_dynamic_{engine}.json"
        return json.loads(path.read_text(encoding="utf-8"))

    def copy_minimal_package(self, destination):
        destination.mkdir(parents=True)
        (destination / "scripts").mkdir()
        (destination / "evidence").mkdir()
        for name in (
            "dynamic_evidence_contract.py",
            "make_validation_md.py",
            "validate_dynamic_physics.py",
        ):
            shutil.copy2(SCRIPT_DIR / name, destination / "scripts" / name)
        for source in (PACKAGE_DIR / "evidence").rglob("*.json"):
            target = destination / "evidence" / source.relative_to(PACKAGE_DIR / "evidence")
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(source, target)
        for source in PACKAGE_DIR.rglob("*"):
            if source.is_file() and source.suffix.lower() in {".usd", ".usda", ".usdc"}:
                target = destination / source.relative_to(PACKAGE_DIR)
                target.parent.mkdir(parents=True, exist_ok=True)
                shutil.copy2(source, target)

    def run_generator(self, package):
        return subprocess.run(
            [sys.executable, str(package / "scripts/make_validation_md.py")],
            capture_output=True,
            text=True,
            check=False,
        )

    def test_early_failure_removes_preexisting_pass_report(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "dynamic.json"
            output.write_text('{"passed": true, "marker": "stale"}\n', encoding="utf-8")
            result = subprocess.run(
                [
                    sys.executable,
                    str(SCRIPT_DIR / "validate_dynamic_physics.py"),
                    str(Path(directory) / "missing.usda"),
                    "newton",
                    str(output),
                ],
                capture_output=True,
                text=True,
                check=False,
            )
            self.assertNotEqual(result.returncode, 0)
            self.assertFalse(output.exists())

    def test_post_app_exception_removes_output_and_closes_nonzero(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "dynamic.json"
            output.write_text('{"passed": true}\n', encoding="utf-8")
            runner = textwrap.dedent(
                f"""
                import runpy, sys, types
                from pathlib import Path
                calls = []
                class FakeApp:
                    def __init__(self, *args, **kwargs): pass
                    def close(self, **kwargs): calls.append(kwargs)
                fake = types.ModuleType('isaacsim')
                fake.SimulationApp = FakeApp
                sys.modules['isaacsim'] = fake
                sys.path.insert(0, {str(SCRIPT_DIR)!r})
                sys.argv = [
                    'validate_dynamic_physics.py',
                    {str(PACKAGE_DIR / '00-arm-rs_asm-v3.usda')!r},
                    'newton',
                    {str(output)!r},
                ]
                try:
                    runpy.run_path({str(SCRIPT_DIR / 'validate_dynamic_physics.py')!r}, run_name='__main__')
                except ModuleNotFoundError:
                    pass
                assert calls == [{{'exit_code': 1}}], calls
                assert not Path({str(output)!r}).exists()
                """
            )
            result = subprocess.run(
                [sys.executable, "-c", runner],
                env={"ISAACSIM_PATH": "/tmp"},
                capture_output=True,
                text=True,
                check=False,
            )
            self.assertEqual(result.returncode, 0, result.stderr)

    def test_atomic_write_failure_leaves_no_canonical_or_temporary_output(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "dynamic.json"
            with mock.patch(
                "dynamic_evidence_contract.os.replace",
                side_effect=OSError("injected replace failure"),
            ):
                with self.assertRaisesRegex(OSError, "injected replace failure"):
                    atomic_write_json(output, {"passed": True})
            self.assertFalse(output.exists())
            self.assertEqual(list(Path(directory).iterdir()), [])

    def test_current_reports_satisfy_shared_contract(self):
        for engine in ("newton", "physx"):
            with self.subTest(engine=engine):
                self.assertEqual(dynamic_evidence_problems(self.load_report(engine), engine), [])

    def test_shared_contract_rejects_adversarial_mutations(self):
        mutations = {
            "position flag": {"positions_within_limits": False},
            "step count": {"physics_steps_advanced": 1},
            "published hold metric": {"max_angular_hold_error_rad": 1.518},
            "passive motion": {"passive_motion_joint3_rad": 0.0},
            "widened tolerance": {"hold_error_tolerances": [10.0] * 8},
            "non-finite start": {"start_positions": [float("inf")] * 8},
        }
        original = self.load_report("newton")
        for name, values in mutations.items():
            with self.subTest(mutation=name):
                report = deepcopy(original)
                report.update(values)
                self.assertNotEqual(dynamic_evidence_problems(report, "newton"), [])

    def test_shared_contract_rejects_coherent_settling_continuity_bypass(self):
        report = deepcopy(self.load_report("newton"))
        for key in (
            "hold_start_positions",
            "hold_end_positions",
            "hold_min_positions",
            "hold_max_positions",
        ):
            report[key][0] = 0.006
        report["hold_max_abs_error_by_dof"][0] = 0.006
        report["hold_max_excursion_by_dof"][0] = 0.0
        report["max_angular_hold_error_rad"] = max(
            report["hold_max_abs_error_by_dof"][:6]
        )
        report["max_angular_hold_excursion_rad"] = max(
            report["hold_max_excursion_by_dof"][:6]
        )
        report["observed_max_positions"][0] = 0.006
        report["passive_probe_positions_joint3"] = [
            value + 0.05 for value in report["passive_probe_positions_joint3"]
        ]
        trace = report["passive_probe_positions_joint3"]
        report["passive_motion_joint3_rad"] = trace[-1] - trace[0]
        report["validation_errors"] = []
        report["passed"] = True
        self.assertNotEqual(dynamic_evidence_problems(report, "newton"), [])

    def test_generator_rejects_contradictory_pass_evidence(self):
        with tempfile.TemporaryDirectory() as directory:
            package = Path(directory) / PACKAGE_DIR.name
            self.copy_minimal_package(package)
            path = package / "evidence/physics_fidelity_dynamic_newton.json"
            report = json.loads(path.read_text(encoding="utf-8"))
            report.update(
                {
                    "positions_within_limits": False,
                    "physics_steps_advanced": 1,
                    "max_angular_hold_error_rad": 1.518,
                    "passive_motion_joint3_rad": 0.0,
                    "passed": True,
                }
            )
            path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
            result = self.run_generator(package)
            self.assertNotEqual(result.returncode, 0)
            self.assertFalse((package / "VALIDATION.md").exists())

    def test_generator_rejects_coherent_settling_continuity_bypass(self):
        with tempfile.TemporaryDirectory() as directory:
            package = Path(directory) / PACKAGE_DIR.name
            self.copy_minimal_package(package)
            path = package / "evidence/physics_fidelity_dynamic_newton.json"
            report = json.loads(path.read_text(encoding="utf-8"))
            for key in (
                "hold_start_positions",
                "hold_end_positions",
                "hold_min_positions",
                "hold_max_positions",
            ):
                report[key][0] = 0.006
            report["hold_max_abs_error_by_dof"][0] = 0.006
            report["hold_max_excursion_by_dof"][0] = 0.0
            report["max_angular_hold_error_rad"] = max(
                report["hold_max_abs_error_by_dof"][:6]
            )
            report["max_angular_hold_excursion_rad"] = max(
                report["hold_max_excursion_by_dof"][:6]
            )
            report["observed_max_positions"][0] = 0.006
            report["passive_probe_positions_joint3"] = [
                value + 0.05 for value in report["passive_probe_positions_joint3"]
            ]
            trace = report["passive_probe_positions_joint3"]
            report["passive_motion_joint3_rad"] = trace[-1] - trace[0]
            report["validation_errors"] = []
            report["passed"] = True
            path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
            result = self.run_generator(package)
            self.assertNotEqual(result.returncode, 0)
            self.assertFalse((package / "VALIDATION.md").exists())

    def test_generator_rejects_each_provenance_hash_mutation(self):
        for key in ("validator_sha256", "contract_sha256", "asset_package_sha256"):
            with self.subTest(key=key), tempfile.TemporaryDirectory() as directory:
                package = Path(directory) / PACKAGE_DIR.name
                self.copy_minimal_package(package)
                path = package / "evidence/physics_fidelity_dynamic_newton.json"
                report = json.loads(path.read_text(encoding="utf-8"))
                report[key] = "0" * 64
                path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
                result = self.run_generator(package)
                self.assertNotEqual(result.returncode, 0)
                self.assertFalse((package / "VALIDATION.md").exists())

    def test_generator_invalidates_existing_document_on_each_missing_input(self):
        required = (
            "gt_pj_new_newton.json",
            "gt_pj_new_physx.json",
            "gravity_droop.json",
            "physics_fidelity_validation.json",
            "physics_fidelity_dynamic_newton.json",
            "physics_fidelity_dynamic_physx.json",
        )
        for name in required:
            with self.subTest(name=name), tempfile.TemporaryDirectory() as directory:
                package = Path(directory) / PACKAGE_DIR.name
                self.copy_minimal_package(package)
                shutil.copy2(PACKAGE_DIR / "VALIDATION.md", package / "VALIDATION.md")
                (package / "evidence" / name).unlink()
                result = self.run_generator(package)
                self.assertNotEqual(result.returncode, 0)
                self.assertFalse((package / "VALIDATION.md").exists())

    def test_generator_invalidates_existing_document_when_contract_import_fails(self):
        with tempfile.TemporaryDirectory() as directory:
            package = Path(directory) / PACKAGE_DIR.name
            self.copy_minimal_package(package)
            shutil.copy2(PACKAGE_DIR / "VALIDATION.md", package / "VALIDATION.md")
            (package / "scripts/dynamic_evidence_contract.py").write_text(
                "this is not valid python !!!\n", encoding="utf-8"
            )
            result = self.run_generator(package)
            self.assertNotEqual(result.returncode, 0)
            self.assertFalse((package / "VALIDATION.md").exists())

    def test_generator_rejects_semantically_incomplete_inputs(self):
        mutations = (
            ("gt_pj_new_newton.json", lambda data: data["joint_metrics"].pop("joint1")),
            ("gravity_droop.json", lambda data: data.clear()),
            ("physics_fidelity_validation.json", lambda data: data.update({"passed": False})),
        )
        for name, mutate in mutations:
            with self.subTest(name=name), tempfile.TemporaryDirectory() as directory:
                package = Path(directory) / PACKAGE_DIR.name
                self.copy_minimal_package(package)
                path = package / "evidence" / name
                data = json.loads(path.read_text(encoding="utf-8"))
                mutate(data)
                path.write_text(json.dumps(data, indent=2) + "\n", encoding="utf-8")
                result = self.run_generator(package)
                self.assertNotEqual(result.returncode, 0)
                self.assertFalse((package / "VALIDATION.md").exists())

    def test_generator_is_clean_checkout_reproducible(self):
        generator_source = (SCRIPT_DIR / "make_validation_md.py").read_text(encoding="utf-8")
        self.assertNotIn("/home/", generator_source)
        for name in ("gt_pj_new_newton.json", "gt_pj_new_physx.json"):
            data = json.loads((PACKAGE_DIR / "evidence" / name).read_text(encoding="utf-8"))
            self.assertFalse(Path(data["usd"]).is_absolute())
        with tempfile.TemporaryDirectory() as directory:
            package = Path(directory) / PACKAGE_DIR.name
            self.copy_minimal_package(package)
            result = self.run_generator(package)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertEqual(
                (package / "VALIDATION.md").read_bytes(),
                (PACKAGE_DIR / "VALIDATION.md").read_bytes(),
            )


if __name__ == "__main__":
    unittest.main()
