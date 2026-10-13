#!/usr/bin/env python3
"""
Download GEAR-SONIC model checkpoints and training data from Hugging Face Hub.

Repository: https://huggingface.co/nvidia/GEAR-SONIC

Usage:
    python download_from_hf.py                    # ONNX models for deployment
    python download_from_hf.py --low-latency      # Low-latency ONNX models
    python download_from_hf.py --sonic-v1-1       # SONIC v1.1 ONNX models
    python download_from_hf.py --training          # PyTorch checkpoint + SMPL data
    python download_from_hf.py --sample            # Sample data only (quick start)
    python download_from_hf.py --output-dir /path  # custom output directory
    python download_from_hf.py --no-planner        # skip planner model

A3 (AgiBot A3 / A3 Ultra, SONIC 035 step-200k artifacts from sonic-for-a3/sonic):
    python download_from_hf.py --robot a3 --component pt        # PyTorch checkpoint
    python download_from_hf.py --robot a3 --component onnx rknn # deployment models
    python download_from_hf.py --robot a3 --component sysroot   # RK3588 cross sysroot
    python download_from_hf.py --component all                  # everything (implies A3)
    python download_from_hf.py --robot a3 --component all --list
  A3 downloads are driven by a3_hf_manifest.json; every file is size/SHA-256 checked.
"""

import argparse
import hashlib
import json
import os
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

REPO_ID = "nvidia/GEAR-SONIC"
RELEASE_MANIFEST_FILE = "config.json"
RELEASE_MANIFEST_SCHEMA_VERSION = 1

# (filename in HF repo, local destination relative to output_dir)
POLICY_FILES = [
    ("model_encoder.onnx", "policy/release/model_encoder.onnx"),
    ("model_decoder.onnx", "policy/release/model_decoder.onnx"),
    ("observation_config.yaml", "policy/release/observation_config.yaml"),
]

LOW_LATENCY_POLICY_FILES = [
    ("low_latency/model_encoder.onnx", "policy/low_latency/model_encoder.onnx"),
    ("low_latency/model_decoder.onnx", "policy/low_latency/model_decoder.onnx"),
    ("low_latency/observation_config.yaml", "policy/low_latency/observation_config.yaml"),
]

SONIC_V1_1_POLICY_FILES = [
    ("sonic_v1_1/model_encoder.onnx", "policy/sonic_v1_1/model_encoder.onnx"),
    ("sonic_v1_1/model_decoder.onnx", "policy/sonic_v1_1/model_decoder.onnx"),
    (
        "sonic_v1_1/observation_config.yaml",
        "policy/sonic_v1_1/observation_config.yaml",
    ),
]

PLANNER_FILE = ("planner_sonic.onnx", "planner/target_vel/V2/planner_sonic.onnx")

TRAINING_FILES = [
    ("sonic_release/last.pt", "sonic_release/last.pt"),
    ("sonic_release/config.yaml", "sonic_release/config.yaml"),
]

LOW_LATENCY_TRAINING_FILES = [
    ("low_latency/last.pt", "low_latency/last.pt"),
    ("low_latency/config.yaml", "low_latency/config.yaml"),
    ("low_latency/model_config.yaml", "low_latency/model_config.yaml"),
]

SONIC_V1_1_TRAINING_FILES = [
    ("sonic_v1_1/last.pt", "sonic_v1_1/last.pt"),
    ("sonic_v1_1/config.yaml", "sonic_v1_1/config.yaml"),
    ("sonic_v1_1/model_config.yaml", "sonic_v1_1/model_config.yaml"),
]

SMPL_TAR_PARTS_PREFIX = "bones_seed_smpl/bones_seed_smpl.tar.part_"
SMPL_TAR_PARTS = [f"{SMPL_TAR_PARTS_PREFIX}a{c}" for c in "abcdefg"]


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="Download GEAR-SONIC checkpoints from Hugging Face Hub"
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=None,
        help=(
            "Directory to save files. "
            "Defaults to gear_sonic_deploy/ (deploy) or repo root (training)."
        ),
    )
    parser.add_argument(
        "--no-planner",
        action="store_true",
        help="Skip downloading the kinematic planner ONNX model",
    )
    parser.add_argument(
        "--training",
        action="store_true",
        help="Download training checkpoint + SMPL motion data (~30 GB)",
    )
    variant_group = parser.add_mutually_exclusive_group()
    variant_group.add_argument(
        "--low-latency",
        action="store_true",
        help=(
            "Download the low-latency SONIC variant. For deployment, files are "
            "placed under gear_sonic_deploy/policy/low_latency/. With --training, "
            "downloads low_latency/last.pt and its configs."
        ),
    )
    variant_group.add_argument(
        "--sonic-v1-1",
        dest="sonic_v1_1",
        action="store_true",
        help=(
            "Download the SONIC v1.1 teleoperation variant with "
            "robot-heading-normalized targets and "
            "wrist-pose augmentation. For deployment, files are placed under "
            "gear_sonic_deploy/policy/sonic_v1_1/. With --training, downloads "
            "sonic_v1_1/last.pt and its configs."
        ),
    )
    parser.add_argument(
        "--sample",
        action="store_true",
        help="Download sample motion data only (1 walking sequence, ~4 MB)",
    )
    parser.add_argument(
        "--no-smpl",
        action="store_true",
        help="With --training, skip SMPL data download (checkpoint only)",
    )
    parser.add_argument(
        "--token",
        default=None,
        help="Hugging Face token (or set HF_TOKEN env var / run `hf auth login`)",
    )
    return parser.parse_args(argv)


def _ensure_huggingface_hub():
    try:
        from huggingface_hub import hf_hub_download, snapshot_download
        return hf_hub_download, snapshot_download
    except ImportError:
        print("huggingface_hub is not installed. Install it with:")
        print("  pip install huggingface_hub")
        sys.exit(1)


def load_release_manifest(hf_hub_download, repo_id, token=None):
    """Download and validate the repository-level release manifest."""
    print(f"  Downloading {RELEASE_MANIFEST_FILE} ...", flush=True)
    cached = hf_hub_download(
        repo_id=repo_id,
        filename=RELEASE_MANIFEST_FILE,
        token=token,
    )
    try:
        manifest = json.loads(Path(cached).read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError(f"invalid {RELEASE_MANIFEST_FILE}: {exc}") from exc

    if not isinstance(manifest, dict):
        raise ValueError(f"{RELEASE_MANIFEST_FILE} must contain a JSON object")
    if manifest.get("schema_version") != RELEASE_MANIFEST_SCHEMA_VERSION:
        raise ValueError(
            f"unsupported {RELEASE_MANIFEST_FILE} schema_version: "
            f"{manifest.get('schema_version')!r}"
        )
    if manifest.get("model_id") != repo_id:
        raise ValueError(
            f"{RELEASE_MANIFEST_FILE} model_id is {manifest.get('model_id')!r}, "
            f"expected {repo_id!r}"
        )

    print(
        f"  -> validated schema {manifest['schema_version']} for {manifest['model_id']}"
    )
    return manifest


def validate_variant_files(manifest, variant, group, expected_entries):
    """Ensure a downloader file group matches the published release manifest."""
    try:
        actual_files = manifest["variants"][variant][group]
    except (KeyError, TypeError) as exc:
        raise ValueError(
            f"{RELEASE_MANIFEST_FILE} is missing variants.{variant}.{group}"
        ) from exc

    expected_files = [remote_path for remote_path, _ in expected_entries]
    if actual_files != expected_files:
        raise ValueError(
            f"{RELEASE_MANIFEST_FILE} variants.{variant}.{group} does not match "
            f"this downloader: expected {expected_files!r}, got {actual_files!r}"
        )


def validate_shared_value(manifest, key, expected_value):
    """Ensure shared release metadata matches the downloader contract."""
    try:
        actual_value = manifest["shared_files"][key]
    except (KeyError, TypeError) as exc:
        raise ValueError(
            f"{RELEASE_MANIFEST_FILE} is missing shared_files.{key}"
        ) from exc

    if actual_value != expected_value:
        raise ValueError(
            f"{RELEASE_MANIFEST_FILE} shared_files.{key} does not match this "
            f"downloader: expected {expected_value!r}, got {actual_value!r}"
        )


def download_file(hf_hub_download, repo_id, hf_filename, local_dest, token=None):
    """Download hf_filename from the Hub and place it at local_dest."""
    print(f"  Downloading {hf_filename} ...", flush=True)
    cached = hf_hub_download(
        repo_id=repo_id,
        filename=hf_filename,
        token=token,
    )
    local_dest.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(cached, local_dest)
    print(f"  -> {local_dest}")


def download_and_extract_smpl(hf_hub_download, repo_id, output_dir, token=None):
    """Download split tar parts and extract SMPL data."""
    parts_dir = output_dir / "bones_seed_smpl"
    parts_dir.mkdir(parents=True, exist_ok=True)

    print(f"  Downloading {len(SMPL_TAR_PARTS)} parts (~30 GB total) ...", flush=True)
    part_paths = []
    for hf_filename in SMPL_TAR_PARTS:
        local_name = Path(hf_filename).name
        local_dest = parts_dir / local_name
        if local_dest.exists():
            print(f"  (cached) {local_name}")
            part_paths.append(local_dest)
            continue
        cached = hf_hub_download(repo_id=repo_id, filename=hf_filename, token=token)
        shutil.copy2(cached, local_dest)
        part_paths.append(local_dest)
        print(f"  Downloaded {local_name}")

    # Reassemble and extract
    data_dir = output_dir / "data"
    data_dir.mkdir(parents=True, exist_ok=True)
    print(f"  Extracting to {data_dir}/smpl_filtered/ ...", flush=True)

    # cat parts | tar xf - -C data/
    cat_cmd = f"cat {parts_dir}/bones_seed_smpl.tar.part_*"
    tar_cmd = f"tar xf - -C {data_dir}"
    result = subprocess.run(
        f"{cat_cmd} | {tar_cmd}",
        shell=True,
        capture_output=True,
        text=True,
    )
    if result.returncode != 0:
        print(f"  ERROR: Extraction failed: {result.stderr}")
        sys.exit(1)

    # Count extracted files
    smpl_dir = data_dir / "smpl_filtered"
    if smpl_dir.exists():
        n_files = sum(1 for f in smpl_dir.iterdir() if f.suffix == ".pkl")
        print(f"  -> {smpl_dir} ({n_files} PKL files)")
    else:
        print(f"  WARNING: Expected {smpl_dir} but directory not found")

    # Clean up tar parts
    print("  Cleaning up tar parts ...")
    shutil.rmtree(parts_dir)


def download_sample_data(snapshot_download, repo_id, output_dir, token=None):
    """Download sample motion data (1 walking sequence)."""
    print("  Downloading sample data ...", flush=True)
    snapshot_download(
        repo_id=repo_id,
        allow_patterns="sample_data/*",
        local_dir=str(output_dir),
        token=token,
    )
    sample_dir = output_dir / "sample_data"
    if sample_dir.exists():
        n_files = sum(1 for _ in sample_dir.rglob("*.pkl"))
        print(f"  -> {sample_dir} ({n_files} PKL files)")


def main_gear_sonic(argv=None):
    args = parse_args(argv)
    if args.sample and (args.low_latency or args.sonic_v1_1):
        print(
            "ERROR: model variant flags cannot be combined with --sample",
            file=sys.stderr,
        )
        sys.exit(2)

    hf_hub_download, snapshot_download = _ensure_huggingface_hub()
    if args.low_latency:
        variant = "low_latency"
    elif args.sonic_v1_1:
        variant = "sonic_v1_1"
    else:
        variant = "default"

    repo_root = Path(__file__).resolve().parent

    if args.training or args.sample:
        output_dir = args.output_dir if args.output_dir else repo_root
    else:
        output_dir = args.output_dir if args.output_dir else repo_root / "gear_sonic_deploy"

    print("=" * 60)
    print("  GEAR-SONIC — Hugging Face Model Downloader")
    print(f"  Repository : {REPO_ID}")
    print(f"  Output dir : {output_dir}")
    if args.training:
        print(f"  Mode       : {variant.replace('_', '-')} training checkpoint")
    elif args.sample:
        print(f"  Mode       : sample data (quick start)")
    else:
        print(f"  Mode       : {variant.replace('_', '-')} deployment (ONNX models)")
    print("=" * 60)

    print("\n[Release Manifest]")
    try:
        manifest = load_release_manifest(
            hf_hub_download, REPO_ID, token=args.token
        )
        if args.sample:
            validate_shared_value(manifest, "sample_data_prefix", "sample_data/")
        elif args.training:
            training_files = {
                "default": TRAINING_FILES,
                "low_latency": LOW_LATENCY_TRAINING_FILES,
                "sonic_v1_1": SONIC_V1_1_TRAINING_FILES,
            }[variant]
            validate_variant_files(
                manifest, variant, "training_files", training_files
            )
            if not args.low_latency and not args.no_smpl:
                validate_shared_value(
                    manifest, "smpl_archive_parts", SMPL_TAR_PARTS
                )
        else:
            policy_files = {
                "default": POLICY_FILES,
                "low_latency": LOW_LATENCY_POLICY_FILES,
                "sonic_v1_1": SONIC_V1_1_POLICY_FILES,
            }[variant]
            validate_variant_files(
                manifest, variant, "deployment_files", policy_files
            )
            if not args.no_planner:
                validate_shared_value(manifest, "planner", PLANNER_FILE[0])
    except ValueError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        sys.exit(1)

    if args.sample:
        print("\n[Sample Data]")
        download_sample_data(snapshot_download, REPO_ID, output_dir, token=args.token)

    elif args.training:
        print("\n[Checkpoint]")
        for hf_filename, local_rel in training_files:
            download_file(
                hf_hub_download, REPO_ID, hf_filename,
                output_dir / local_rel, token=args.token,
            )

        if args.low_latency:
            print("\n[SMPL Motion Data] Skipped (not part of low-latency checkpoint download)")
        elif not args.no_smpl:
            print("\n[SMPL Motion Data]")
            download_and_extract_smpl(hf_hub_download, REPO_ID, output_dir, token=args.token)
        else:
            print("\n[SMPL Motion Data] Skipped (--no-smpl)")

    else:
        print("\n[Policy]")
        for hf_filename, local_rel in policy_files:
            download_file(
                hf_hub_download, REPO_ID, hf_filename,
                output_dir / local_rel, token=args.token,
            )

        if not args.no_planner:
            print("\n[Planner]")
            hf_filename, local_rel = PLANNER_FILE
            download_file(
                hf_hub_download, REPO_ID, hf_filename,
                output_dir / local_rel, token=args.token,
            )

    print("\n" + "=" * 60)
    print("  Done! Files saved under:")
    print(f"  {output_dir}")
    print("=" * 60)


# ---------------------------------------------------------------------------
# A3 (sonic-for-a3/sonic) manifest-driven downloader
# ---------------------------------------------------------------------------
ROOT = Path(__file__).resolve().parent


def matches(path, entry):
    if not path.is_file() or path.stat().st_size != entry['size']:
        return False
    sha = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            sha.update(chunk)
    return sha.hexdigest() == entry['sha256']


def install_file(entry, output_dir, download, repo_id, revision, token=None, force=False):
    destination = output_dir / entry['destination']
    if not force and matches(destination, entry):
        print(f"[verified] {entry['destination']}")
        return
    cached = Path(download(repo_id=repo_id, filename=entry['remote'], revision=revision,
                           token=token, force_download=force))
    if not matches(cached, entry):
        raise RuntimeError(f"Size/SHA-256 mismatch: {entry['remote']}; retry with --force")
    destination.parent.mkdir(parents=True, exist_ok=True)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(dir=destination.parent, prefix='.hf-', delete=False) as f:
            temporary = Path(f.name)
            with cached.open('rb') as source:
                shutil.copyfileobj(source, f)
        if not matches(temporary, entry):
            raise RuntimeError(f"Copy verification failed: {destination}")
        temporary.chmod(0o644)
        os.replace(temporary, destination)
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)
    print(f"[downloaded] {entry['destination']}")


def selected_entries(manifest, components):
    groups = set(components)
    if 'all' in groups:
        groups.update(['pt', 'onnx', 'rknn', 'sysroot'])
    if 'runtime' in groups:
        groups.update(['onnx', 'rknn'])
    # One shared release configuration for model downloads. This is an ordinary
    # verified artifact; no tracking ping or forced cache bypass is performed.
    if groups.intersection({'pt', 'onnx', 'rknn'}):
        groups.add('model_metadata')
    return [e for e in manifest['files'] if e['group'] in groups]


def main_a3(argv=None):
    manifest = json.loads((ROOT / 'a3_hf_manifest.json').read_text())
    parser = argparse.ArgumentParser(
        description='Download A3 035 / 200k artifacts into their configured source-tree paths.',
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--component', nargs='+', choices=['pt', 'onnx', 'rknn', 'sysroot', 'runtime', 'all'],
                        default=['runtime'], help='one or more groups; default: runtime (ONNX + RKNN)')
    parser.add_argument('--output-dir', type=Path, default=ROOT, help='source-tree root to populate')
    parser.add_argument('--repo-id', default=os.environ.get('A3_HF_REPO_ID', manifest['repo_id']))
    parser.add_argument('--revision', default=os.environ.get('A3_HF_REVISION', manifest['revision']),
                        help='defaults to the pinned release commit; checksums always remain enforced')
    parser.add_argument('--token', default=os.environ.get('HF_TOKEN'))
    parser.add_argument('--force', action='store_true', help='redownload and replace even verified files')
    parser.add_argument('--list', action='store_true', help='show selected files without downloading')
    args = parser.parse_args(argv)
    entries = selected_entries(manifest, args.component)
    print(f"Repository: {args.repo_id} @ {args.revision}")
    print(f"Selected: {len(entries)} files, {sum(e['size'] for e in entries) / 1e6:.1f} MB")
    if args.list:
        for e in entries:
            print(f"{e['group']}: {e['remote']} -> {e['destination']}")
        return 0
    try:
        from huggingface_hub import hf_hub_download
    except ImportError:
        parser.error('Install the dependency: python -m pip install huggingface_hub')
    for entry in entries:
        install_file(entry, args.output_dir, hf_hub_download, args.repo_id, args.revision, args.token, args.force)
    return 0


# ---------------------------------------------------------------------------
# Unified entry point
# ---------------------------------------------------------------------------
_A3_ROBOTS = ("a3", "a3_ultra")
_A3_ONLY_FLAGS = ("--component", "--list", "--force", "--revision", "--repo-id")


def _extract_robot(argv):
    """Return (robot_or_None, argv_without_--robot)."""
    robot = None
    out = []
    it = iter(range(len(argv)))
    i = 0
    while i < len(argv):
        a = argv[i]
        if a == "--robot":
            robot = argv[i + 1] if i + 1 < len(argv) else None
            i += 2
            continue
        if a.startswith("--robot="):
            robot = a.split("=", 1)[1]
            i += 1
            continue
        out.append(a)
        i += 1
    return robot, out


def main(argv=None):
    """Dispatch between the G1 (nvidia/GEAR-SONIC) and A3 (sonic-for-a3/sonic) flows."""
    argv = list(sys.argv[1:] if argv is None else argv)
    robot, rest = _extract_robot(argv)
    a3_flags = any(a == f or a.startswith(f + "=") for a in rest for f in _A3_ONLY_FLAGS)
    if robot in _A3_ROBOTS or (robot is None and a3_flags):
        if robot == "a3_ultra":
            print(
                "[note] a3_ultra deployment reuses the A3 035 step-200k artifacts "
                "(hardware A3/A3U share legs/waist/arms); fingers are not driven."
            )
        return main_a3(rest)
    if robot not in (None, "g1"):
        print(
            f"ERROR: no published pretrained artifacts for --robot {robot}; "
            f"choices with downloads: g1 (default), {', '.join(_A3_ROBOTS)}.",
            file=sys.stderr,
        )
        return 2
    return main_gear_sonic(rest)


if __name__ == "__main__":
    raise SystemExit(main())
