"""Read-only inventory behavior with package/GPU subprocesses stubbed."""

import subprocess
import sys
import tempfile
from pathlib import Path
import unittest
from unittest.mock import Mock, patch

sys.path.insert(
    0, str(Path(__file__).resolve().parents[1] / "skills/spatial-memory/scripts")
)
from detect_simulators import isaac_roots, probe_gpu, probe_python


class SimulatorInventoryTests(unittest.TestCase):
    def test_multiple_gpu_records_preserve_device_driver_pairing(self):
        output = "NVIDIA GeForce RTX 4090, 24564, 580.173.02\nNVIDIA A100, 40960, 580.173.02\n"
        with (
            patch("detect_simulators.shutil.which", return_value="/usr/bin/nvidia-smi"),
            patch("detect_simulators.subprocess.run", return_value=Mock(stdout=output)),
        ):
            report = probe_gpu()
        self.assertEqual(len(report["devices"]), 2)
        self.assertEqual(report["devices"][0]["vram_mib"], 24564)
        self.assertEqual(report["devices"][1]["name"], "NVIDIA A100")

    def test_unavailable_and_failed_gpu_queries_are_distinct(self):
        with patch("detect_simulators.shutil.which", return_value=None):
            self.assertEqual(probe_gpu()["status"], "nvidia-smi unavailable")
        with (
            patch("detect_simulators.shutil.which", return_value="nvidia-smi"),
            patch(
                "detect_simulators.subprocess.run",
                side_effect=subprocess.TimeoutExpired("nvidia-smi", 10),
            ),
        ):
            report = probe_gpu()
        self.assertEqual(report["status"], "query failed")
        self.assertEqual(report["devices"], [])

    def test_package_metadata_does_not_require_simulator_import(self):
        with patch(
            "detect_simulators.subprocess.run",
            return_value=Mock(stdout='{"mujoco": "3.3.0"}'),
        ) as command:
            report = probe_python("/custom/env/bin/python")
        self.assertEqual(report["packages"], {"mujoco": "3.3.0"})
        self.assertEqual(command.call_args.args[0][0], "/custom/env/bin/python")
        with patch(
            "detect_simulators.subprocess.run", side_effect=OSError("Unavailable")
        ):
            self.assertIn("error", probe_python("/missing/python"))

    def test_custom_standalone_installation(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder) / "isaac"
            root.mkdir()
            (root / "isaac-sim.sh").touch()
            (root / "VERSION").write_text("5.1.0")
            found = [item for item in isaac_roots([root]) if item["root"] == str(root)]
        self.assertEqual(found[0]["version"], "5.1.0")


if __name__ == "__main__":
    unittest.main()
