"""Tool contract checks, distinct from real SDK/firmware verification."""
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parent.parent


def wizard(*args):
    return subprocess.run([sys.executable, str(ROOT / "tools/setup_wizard.py"), *args],
                          text=True, capture_output=True)


class ConfigurationTools(unittest.TestCase):
    def test_measured_two_wheel_config(self):
        result = wizard("--two-wheel", "--left-offset", "4.25", "--side-offset", "-2")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("config.leftOffset = 4.25;", result.stdout)
        self.assertIn("config.useRight = false;", result.stdout)
        self.assertIn("config.useImu = true;", result.stdout)

    def test_impossible_and_nonfinite_geometry(self):
        for args in [("--two-wheel", "--no-imu"), ("--diameter", "0"),
                     ("--side-offset", "nan"), ("--left-offset", "inf"),
                     ("--no-imu", "--left-offset", "5", "--right-offset", "5")]:
            with self.subTest(args=args):
                self.assertNotEqual(wizard(*args).returncode, 0)

    def test_generator_never_overwrites_existing_directory(self):
        with tempfile.TemporaryDirectory() as directory:
            marker = Path(directory) / "keep.txt"
            marker.write_text("user work")
            result = subprocess.run([sys.executable, str(ROOT / "tools/create_pros_project.py"), directory],
                                    text=True, capture_output=True)
            self.assertNotEqual(result.returncode, 0)
            self.assertIn("destination already exists", result.stderr)
            self.assertEqual(marker.read_text(), "user work")

    def test_generator_copies_sources_after_cli_success(self):
        # A fake CLI only validates subprocess/copy behavior. It is NOT a PROS smoke test.
        with tempfile.TemporaryDirectory() as directory:
            base = Path(directory)
            cli = base / "pros"
            cli.write_text(f"#!{sys.executable}\n"
                           "import pathlib,sys\n"
                           "assert sys.argv[1:3] == ['conductor','new-project']\n"
                           "assert sys.argv[4:] == ['v5','--kernel','4.1.1']\n"
                           "p=pathlib.Path(sys.argv[3]); p.mkdir()\n"
                           "(p/'include').mkdir(); (p/'src').mkdir()\n"
                           "(p/'project.pros').write_text('{}')\n")
            cli.chmod(0o755)
            destination = base / "robot"
            process_env = os.environ.copy()
            process_env["PATH"] = str(base) + os.pathsep + process_env.get("PATH", "")
            result = subprocess.run([sys.executable, str(ROOT / "tools/create_pros_project.py"),
                                     str(destination)], env=process_env, text=True, capture_output=True)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertEqual((destination / "src/main.cpp").read_bytes(),
                             (ROOT / "examples/pros/main.cpp").read_bytes())
            self.assertTrue((destination / "include/ovelo/localization.hpp").is_file())
            self.assertEqual((destination / "include/main.h").read_bytes(),
                             (ROOT / "examples/pros/main.h").read_bytes())


if __name__ == "__main__":
    unittest.main()
