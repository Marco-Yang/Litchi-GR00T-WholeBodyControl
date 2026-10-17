#!/usr/bin/env python3
"""Pack the robot-side ZMQ relay (route C) into a small tarball for the MDU: only what ``a3_zmq_relay`` needs, no models / motions / policy.

    python3 scripts/a3_make_relay_bundle.py --pkg dist/a3_deploy_rockchip [--out dist/a3_relay_rockchip.tar.gz] [--check]

The file set is computed, not hand-listed: starting from ``a3_zmq_relay`` and the AimRT plugins named in ``config/a3_aimrt_config.iceoryx.yaml``
it follows the DT_NEEDED entries (``readelf -d``, works for any architecture) through the package directory and includes every library that
is in the package.  What is NOT in the package is printed as "must come from the MDU" (libc / libstdc++, ROS 2 from /opt/ros, and libzmq.so.5
-- the package does not bundle it, plan 18.20).  Nothing is executed from the package and nothing outside ``--out`` is written.

The tarball keeps the package layout (``a3_relay_<arch>/...``), symlinks included, and gets a MANIFEST.sha256 inside plus a ``<out>.sha256`` file.
"""

from __future__ import annotations

import argparse
import hashlib
from pathlib import Path
import re
import subprocess
import sys
import tarfile

FIXED_FILES = ["run_a3.sh", "run_a3_relay.sh", "a3_real_preflight.sh", "a3_relay_tool.py"]
CONFIG_DIR = "config"
ROOT_BINARY = "a3_zmq_relay"
PLUGIN_CONFIG = "config/a3_aimrt_config.iceoryx.yaml"


def needed(path: Path) -> list[str]:
    out = subprocess.run(["readelf", "-d", str(path)], capture_output=True, text=True)
    if out.returncode != 0:
        raise RuntimeError(f"readelf failed on {path}: {out.stderr.strip()}")
    return re.findall(r"\(NEEDED\)\s+Shared library: \[([^\]]+)\]", out.stdout)


def plugin_names(cfg: Path) -> list[str]:
    return re.findall(r"^\s*path:\s*(lib[\w.+-]+\.so[\w.]*)\s*$", cfg.read_text(), flags=re.M)


def closure(pkg: Path) -> tuple[set[Path], set[str]]:
    """(files of the package the relay needs, sonames that must come from the target system)."""
    roots = [pkg / ROOT_BINARY] + [pkg / n for n in plugin_names(pkg / PLUGIN_CONFIG)]
    missing_roots = [r.name for r in roots if not r.exists()]
    if missing_roots:
        raise FileNotFoundError(f"not in the package: {', '.join(missing_roots)}")
    files: set[Path] = set()
    external: set[str] = set()
    todo = list(roots)
    while todo:
        f = todo.pop()
        real = f.resolve()
        if f in files:
            continue
        files.add(f)
        files.add(real)
        for soname in needed(real):
            cand = pkg / soname
            if cand.exists():
                todo.append(cand)
            else:
                external.add(soname)
    return files, external


def sha256(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as fh:
        for chunk in iter(lambda: fh.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def build(pkg: Path, out: Path | None, top: str) -> tuple[list[Path], set[str]]:
    files, external = closure(pkg)
    for name in FIXED_FILES:
        if (pkg / name).exists():
            files.add(pkg / name)
    cfg = pkg / CONFIG_DIR
    if cfg.is_dir():
        files.update(p for p in cfg.rglob("*") if p.is_file())
    members = sorted(files)
    if out is not None:
        out.parent.mkdir(parents=True, exist_ok=True)
        manifest = "".join(f"{sha256(p.resolve())}  {top}/{p.relative_to(pkg)}\n" for p in members if not p.is_symlink())
        with tarfile.open(out, "w:gz") as tf:
            for p in members:
                tf.add(p, arcname=f"{top}/{p.relative_to(pkg)}", recursive=False)
            import io

            data = manifest.encode()
            info = tarfile.TarInfo(f"{top}/MANIFEST.sha256")
            info.size = len(data)
            info.mode = 0o644
            tf.addfile(info, io.BytesIO(data))
        Path(str(out) + ".sha256").write_text(f"{sha256(out)}  {out.name}\n")
    return members, external


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--pkg", type=Path, required=True, help="a built package directory (dist/a3_deploy_rockchip)")
    ap.add_argument("--out", type=Path, default=None, help="tarball to write (default: <pkg>/../a3_relay_<arch>.tar.gz)")
    ap.add_argument("--check", action="store_true", help="only print the file list and what must come from the target, write nothing")
    args = ap.parse_args()
    pkg = args.pkg.resolve()
    arch = pkg.name.removeprefix("a3_deploy_")
    out = args.out or pkg.parent / f"a3_relay_{arch}.tar.gz"
    members, external = build(pkg, None if args.check else out, f"a3_relay_{arch}")
    total = sum(p.resolve().stat().st_size for p in members if p.exists())
    print(f"package {pkg}: {len(members)} files, {total / 1e6:.1f} MB uncompressed")
    for p in members:
        print(f"  {p.relative_to(pkg)}")
    print("must come from the target machine (not in the package):")
    for s in sorted(external):
        print(f"  {s}")
    if not args.check:
        print(f"wrote {out} ({out.stat().st_size / 1e6:.1f} MB) and {out}.sha256")
    return 0


if __name__ == "__main__":
    sys.exit(main())
