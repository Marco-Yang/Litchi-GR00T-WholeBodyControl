"""One-command Pico teleop of A3 / A3 Ultra in MuJoCo (the A3 counterpart of run_sim_loop.py).

Chain (docs/workflow/a3_integration_plan.md section 9):

    Pico (or replay emulator) --ZMQ "pose":5556--> a3_teleop_retarget_node.py (GMR)
        --ZMQ "a3_ref":5560--> sim2sim_a3_mujoco.py --teleop-zmq  (035 policy, MuJoCo closed loop)

Run inside the ``gmr`` conda env, from a path without spaces (symlink the repo), e.g.::

    PYTHONPATH=. python gear_sonic/scripts/run_a3_teleop_sim.py --robot a3_ultra --source replay \
        --clip synthetic:wave --seconds 25 --report /tmp/a3_teleop.json

``--source pico`` starts no emulator: start the real headset sender yourself
(``pico_manager_thread_server.py`` / ``pico_pose_zmq_minimal.py``) publishing topic "pose" on --pose-port.

For the real robot replace the MuJoCo stage by ``a3_ta_bridge`` (see the plan, section 9.6).
Both ``a3`` and ``a3_ultra`` use the 035 contract (same hardware platform, worklog #37).
"""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

REPO = Path(__file__).resolve().parents[2]
DEFAULT_CKPT = "checkpoints/035_step200000/model_step_200000.pt"
DEFAULT_MJCF = (
    "gear_sonic/data/assets/robot_description/mjcf/"
    "a3_t2d5_loop_passive_foot_twostage_fit_optimized.xml"
)


def _terminate(procs):
    for p in reversed(procs):
        if p.poll() is None:
            p.send_signal(signal.SIGINT)
    deadline = time.time() + 5.0
    for p in procs:
        while p.poll() is None and time.time() < deadline:
            time.sleep(0.1)
        if p.poll() is None:
            p.kill()


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--robot", choices=["a3", "a3_ultra"], default="a3_ultra")
    ap.add_argument("--source", choices=["replay", "pico"], default="replay")
    ap.add_argument("--clip", default="synthetic:stand", help="replay clip: SMPL pkl or synthetic:stand|wave")
    ap.add_argument("--checkpoint", default=DEFAULT_CKPT)
    ap.add_argument("--mjcf", default=DEFAULT_MJCF)
    ap.add_argument("--encoder-mode", choices=["a3_fast", "g1"], default="a3_fast")
    ap.add_argument("--encoder", choices=["a3-fast", "g1", "smpl"], default=None,
                    help="same names as deploy.sh --encoder (a3-fast / g1 = --encoder-mode); smpl is not supported here: "
                         "this runner loads a PT checkpoint with joint-space encoders only (use deploy.sh --encoder smpl)")
    ap.add_argument("--seconds", type=float, default=25.0, help="sim seconds (0 = until the viewer closes)")
    ap.add_argument("--viewer", action="store_true")
    ap.add_argument("--report", default=None, help="JSON report path")
    ap.add_argument("--pose-port", type=int, default=5556)
    ap.add_argument("--ref-port", type=int, default=5560)
    ap.add_argument("--dry-run", action="store_true", help="print the commands and exit")
    args = ap.parse_args(argv)
    if args.encoder == "smpl":
        print("[a3-teleop] --encoder smpl needs an SMPL-encoder policy (none is published for A3 / A3 Ultra); this runner only "
              "loads the 035 PT with the g1 / a3_fast encoders. Use: gear_sonic_deploy/deploy.sh --robot a3_ultra sim "
              "--input-type zmq_manager --encoder smpl --smpl-model <onnx>   (docs/workflow/a3_integration_plan.md section 15)")
        return 2
    if args.encoder:
        args.encoder_mode = args.encoder.replace("-", "_")

    env = dict(os.environ)
    env["PYTHONPATH"] = f"{REPO}{os.pathsep}{env.get('PYTHONPATH', '')}".rstrip(os.pathsep)
    py = sys.executable
    sc = REPO / "gear_sonic" / "scripts"

    stages = []
    if args.source == "replay":
        stages.append(
            ("pico-emulator", [py, str(sc / "pico_pose_zmq_replay.py"), "--clip", args.clip,
                               "--stand-seconds", "6", "--loop", "--port", str(args.pose_port)])
        )
    stages.append(
        ("retarget", [py, str(sc / "a3_teleop_retarget_node.py"),
                      "--pico-port", str(args.pose_port), "--out-port", str(args.ref_port)])
    )
    sim = [py, str(sc / "sim2sim_a3_mujoco.py"), "--checkpoint", args.checkpoint,
           "--encoder-mode", args.encoder_mode, "--mjcf", args.mjcf,
           "--teleop-zmq", f"tcp://localhost:{args.ref_port}", "--teleop-seconds", str(args.seconds)]
    if args.viewer:
        sim.append("--teleop-viewer")
    if args.report:
        sim += ["--teleop-report", args.report]

    print(f"[a3-teleop] robot={args.robot} (035 contract), source={args.source}")
    for name, cmd in stages + [("sim2sim", sim)]:
        print(f"[a3-teleop] {name}: {' '.join(cmd)}")
    if args.dry_run:
        return 0
    if not (REPO / args.checkpoint).exists() and not Path(args.checkpoint).exists():
        print(f"[a3-teleop] checkpoint not found: {args.checkpoint} (python download_from_hf.py --robot a3 --component pt)")
        return 2

    procs = []
    try:
        for name, cmd in stages:
            procs.append(subprocess.Popen(cmd, cwd=REPO, env=env))
            time.sleep(1.5)  # let the PUB sockets bind before the next stage subscribes
        rc = subprocess.run(sim, cwd=REPO, env=env).returncode
    except KeyboardInterrupt:
        rc = 130
    finally:
        _terminate(procs)
    return rc


if __name__ == "__main__":
    raise SystemExit(main())
