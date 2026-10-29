#!/usr/bin/env python3
"""Create a real PROS V5 project, then install the Ovelo headers/example.

The destination must not exist. This leaves the repository Makefile untouched;
PROS supplies the actual kernel, libraries, link script, and firmware build rules.
"""

import argparse
from pathlib import Path
import shutil
import subprocess
import sys

KERNEL_VERSION = "4.1.1"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("destination", type=Path, help="new project directory (must not exist)")
    args = parser.parse_args()
    root = Path(__file__).resolve().parent.parent
    destination = args.destination.expanduser().resolve()
    if destination.exists():
        parser.error("destination already exists; choose a NEW directory to protect your files")
    if root == destination or root in destination.parents:
        parser.error("create the generated project OUTSIDE the Ovelo source checkout")
    if shutil.which("pros") is None:
        parser.error("PROS CLI is missing. Install the official PROS development tools first.")
    if not (root / "include" / "ovelo" / "ovelo.hpp").is_file():
        parser.error("Ovelo headers are missing; run this from a complete Ovelo release")
    command = ["pros", "conductor", "new-project", str(destination),
               "v5", "--kernel", KERNEL_VERSION]
    print("Creating PROS V5 project with kernel", KERNEL_VERSION, flush=True)
    try:
        subprocess.run(command, check=True)
        if not (destination / "project.pros").is_file():
            raise RuntimeError("PROS did not create project.pros; project generation failed")
        shutil.copytree(root / "include" / "ovelo", destination / "include" / "ovelo")
        shutil.copy2(root / "examples" / "pros" / "main.cpp", destination / "src" / "main.cpp")
        shutil.copy2(root / "examples" / "pros" / "main.h", destination / "include" / "main.h")
    except (subprocess.CalledProcessError, OSError, RuntimeError) as exc:
        print(f"Project creation failed: {exc}", file=sys.stderr)
        print("A partial destination may remain. Inspect it before removing it or retrying.",
              file=sys.stderr)
        return 1
    print(f"Created: {destination}")
    print("Edit src/main.cpp ports, geometry, signs, and hardwareConfirmed before use.")
    print("Next: run 'pros make' from the generated project directory.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
