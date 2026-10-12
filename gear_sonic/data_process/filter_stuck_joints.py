"""Filter out pkl motion clips where knee joints are stuck at the lower limit.

83% of SOMA→A3 Ultra retarget clips suffer GMR IK convergence failure: the left
or right knee settles at its lower limit (-7°) instead of tracking the motion.
These clips are harmful for training — they incentivize the policy to hyperextend.

Detection criterion (per clip):
  - L_knee (dof col 22) OR R_knee (dof col 28)
  - std  < STD_THRESH_DEG  (joint barely moves)
  - mean < MEAN_THRESH_DEG (mean is near the lower limit)

Usage:
  # Dry run — print statistics only
  python gear_sonic/data_process/filter_stuck_joints.py \\
      --input data/motion_lib_a3ultra/robot_filtered/

  # Actually delete bad clips
  python gear_sonic/data_process/filter_stuck_joints.py \\
      --input data/motion_lib_a3ultra/robot_filtered/ \\
      --delete
"""

import argparse
import glob
import os

import joblib
import numpy as np

KNEE_L = 22   # dof column: left  knee
KNEE_R = 28   # dof column: right knee

STD_THRESH_DEG  = 0.5   # joint must move at least this much
MEAN_THRESH_DEG = -5.0  # mean must be above this to be considered healthy


def is_stuck(dof: np.ndarray, col: int) -> bool:
    vals = np.degrees(dof[:, col])
    return vals.std() < STD_THRESH_DEG and vals.mean() < MEAN_THRESH_DEG


def scan_file(path: str) -> tuple[bool, str]:
    """Return (is_bad, reason_string)."""
    try:
        data = joblib.load(path)
        inner = list(data.values())[0]
        dof = inner["dof"]  # shape (T, 31)
        lk_stuck = is_stuck(dof, KNEE_L)
        rk_stuck = is_stuck(dof, KNEE_R)
        if lk_stuck and rk_stuck:
            return True, "both_knees_stuck"
        if lk_stuck:
            return True, "L_knee_stuck"
        if rk_stuck:
            return True, "R_knee_stuck"
        return False, "ok"
    except Exception as e:
        return True, f"load_error: {e}"


def main():
    parser = argparse.ArgumentParser(description="Filter stuck-joint clips from motion lib")
    parser.add_argument("--input", required=True, help="Root directory of robot_filtered/ pkl files")
    parser.add_argument("--delete", action="store_true", help="Actually delete bad files (default: dry run)")
    parser.add_argument("--log", default="", help="Optional path to write bad-file list")
    args = parser.parse_args()

    pkls = sorted(glob.glob(os.path.join(args.input, "**", "*.pkl"), recursive=True))
    total = len(pkls)
    print(f"Scanning {total} pkl files in {args.input} ...")

    bad_paths = []
    by_reason: dict[str, int] = {}
    for i, path in enumerate(pkls):
        bad, reason = scan_file(path)
        if bad:
            bad_paths.append(path)
            by_reason[reason] = by_reason.get(reason, 0) + 1
        if (i + 1) % 5000 == 0:
            print(f"  {i+1}/{total} scanned, {len(bad_paths)} bad so far ...")

    good = total - len(bad_paths)
    print(f"\nResults:")
    print(f"  Total:  {total:>7}")
    print(f"  Bad:    {len(bad_paths):>7}  ({100*len(bad_paths)/total:.1f}%)")
    print(f"  Good:   {good:>7}  ({100*good/total:.1f}%)")
    print(f"\nBad breakdown:")
    for reason, count in sorted(by_reason.items(), key=lambda x: -x[1]):
        print(f"  {reason:<25} {count:>6}")

    if args.log:
        with open(args.log, "w") as f:
            for p in bad_paths:
                f.write(p + "\n")
        print(f"\nBad file list written to: {args.log}")

    if args.delete:
        print(f"\nDeleting {len(bad_paths)} bad files ...")
        deleted = 0
        errors = 0
        for path in bad_paths:
            try:
                os.remove(path)
                deleted += 1
            except OSError as e:
                print(f"  ERROR deleting {path}: {e}")
                errors += 1
        print(f"Deleted {deleted} files ({errors} errors).")
        print(f"Remaining clips: {good}")
    else:
        print(f"\nDry run — no files deleted. Re-run with --delete to remove bad clips.")


if __name__ == "__main__":
    main()
