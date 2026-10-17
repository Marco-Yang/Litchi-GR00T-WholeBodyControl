#!/usr/bin/env python3
# ruff: noqa: T201
"""Write a copy of the deploy's runtime config with the per-tick dump switched on and pointed at a file of your choice.

    python3 gear_sonic_deploy/scripts/a3_make_dump_cfg.py --dump /path/run_sim1_dump.bin [--ticks 60000] [--out gear_sonic_deploy/dist/a3_deploy_x86_64/config/a3_runtime_config.dump.yaml]
    A3_RUNTIME_CFG=<the printed --out path> ./deploy.sh ...

Why: the packaged config writes the dump to ONE fixed name (``dump_teleop_real.bin``, also in the simulator), and every new run overwrites it.  This gives each
run its own file and leaves the packaged config alone.  The copy must live next to the packaged config (relative model paths in it are resolved from there).
Only ``logging.dump_ticks`` / ``logging.dump_path`` change; the evaluation recorder (which only records complete reference motions) is left as it is.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import re
import sys

DEFAULT_BASE = Path(__file__).resolve().parents[1] / "dist/a3_deploy_x86_64/config/a3_runtime_config.yaml"


def make(base: Path, dump: Path, ticks: int) -> str:
    text = base.read_text()
    if not re.search(r"(?m)^  dump_ticks: .*$", text) or not re.search(r"(?m)^  dump_path: .*$", text):
        raise SystemExit(f"{base}: no logging.dump_ticks / dump_path lines to replace")
    text = re.sub(r"(?m)^  dump_ticks: .*$", f"  dump_ticks: {ticks}", text)
    text = re.sub(r"(?m)^  dump_path: .*$", lambda _m: f'  dump_path: "{dump}"', text)
    return text


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--dump", type=Path, required=True)
    ap.add_argument("--ticks", type=int, default=60000)
    ap.add_argument("--base", type=Path, default=DEFAULT_BASE)
    ap.add_argument("--out", type=Path, default=DEFAULT_BASE.with_name("a3_runtime_config.dump.yaml"))
    a = ap.parse_args()
    dump = a.dump.resolve()
    if dump.exists():
        print(f"WARNING: {dump} already exists and will be overwritten by the next run: rename it first", file=sys.stderr)
    a.out.write_text(make(a.base, dump, a.ticks))
    print(a.out.resolve())
    return 0


if __name__ == "__main__":
    sys.exit(main())
