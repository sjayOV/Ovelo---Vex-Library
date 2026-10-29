#!/usr/bin/env python3
"""Print a starter Ovelo configuration; never edits files or guesses sensor signs."""
import argparse
import math


def finite_number(value):
    number = float(value)
    if not math.isfinite(number):
        raise argparse.ArgumentTypeError("enter a finite number in inches")
    return number


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--left-offset", type=finite_number, default=5.0)
    parser.add_argument("--right-offset", type=finite_number, default=-5.0)
    parser.add_argument("--side-offset", type=finite_number, default=-3.0)
    parser.add_argument("--diameter", type=finite_number, default=2.75)
    parser.add_argument("--two-wheel", action="store_true", help="left + sideways + IMU")
    parser.add_argument("--no-imu", action="store_true", help="3-wheel encoder heading")
    parser.add_argument("--interactive", action="store_true", help="prompt for measured geometry")
    args = parser.parse_args()
    if args.interactive:
        print("Inches. +X forward, +Y left; use signed wheel offsets.")
        for attr, prompt in [("left_offset", "Left wheel Y"), ("right_offset", "Right wheel Y"),
                             ("side_offset", "Sideways wheel X"), ("diameter", "Tracking wheel diameter")]:
            value = input(f"{prompt} [{getattr(args, attr)}]: ").strip()
            if value:
                try:
                    setattr(args, attr, finite_number(value))
                except (ValueError, argparse.ArgumentTypeError):
                    parser.error(f"{prompt} must be a finite number")
    if args.diameter <= 0:
        parser.error("diameter must be positive")
    if args.two_wheel and args.no_imu:
        parser.error("two tracking wheels require an IMU")
    if args.no_imu and abs(args.left_offset - args.right_offset) <= 1e-6:
        parser.error("encoder heading requires distinct left/right offsets")
    print("// Review all measurements and follow docs/02-sensor-checks.md before motion.")
    print("ovelo::OdometryConfig config;")
    print(f"config.leftOffset = {args.left_offset};")
    print(f"config.rightOffset = {args.right_offset};")
    print(f"config.sideOffset = {args.side_offset};")
    print(f"config.useImu = {'false' if args.no_imu else 'true'};")
    print(f"config.useRight = {'false' if args.two_wheel else 'true'};")
    print(f"constexpr double trackingDiameter = {args.diameter}; // inches")
    print("// Per sensor: ovelo::wheelInches(rawCentidegrees, trackingDiameter)")
    print("// Apply measured +/- sensor signs in the hardware adapter, never abs().")


if __name__ == "__main__":
    main()
