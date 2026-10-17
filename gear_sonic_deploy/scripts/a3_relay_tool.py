#!/usr/bin/env python3
# ruff: noqa: T201
"""Python side of the A3 ZMQ relay wire format (include/a3_relay/a3_relay_wire.hpp) plus three small tools.

    a3_relay_tool.py listen  --host 10.42.10.12 [--state-port 5570] [--seconds 10]
        Prints the relay's state frames (mode, ages, owner, counters, a few joints).  Sends nothing: use it for the first contact with a
        relay, also one started with --listen-only.

    a3_relay_tool.py snapshot --host 10.42.10.12 [--seconds 10]
        Read-only statistics of the state stream at full precision: per slot name, mean / std / min / max of q, mean dq and tau, and both
        IMUs.  Use it to check that the numbers are alive, the slot order and the pose before any command is armed.

    a3_relay_tool.py record --host 10.42.10.12 --seconds 400 --out run.npz
        Read-only full-rate recorder (every state frame, 200 Hz) for diagnosing a real run: wall-clock time of arrival, relay mode / owner /
        accepted+rejected counters, q / dq / tau of all 31 slots and both IMUs.  Saved to --out (.npz) every 30 s and on Ctrl-C, so
        closing the run with Ctrl-C (or the --seconds timeout) keeps everything.  Analyse together with the deploy's per-tick dump:
        gear_sonic_deploy/scripts/a3_analyze_teleop_run.py.

    a3_relay_tool.py fake-pc --host 127.0.0.1 [--hold-s 3] [--then silence|garbage] ...
        A stand-in for the deploy process, to exercise the relay's guard without the deploy: waits for a state frame, then holds the
        MEASURED pose (q_des = measured q, gains --kp / --kd) at --rate Hz for --hold-s seconds, then goes silent (or sends junk).  The
        relay should walk ACTIVE -> HOLD -> DAMP; the tool prints the mode it sees.  It really commands the robot (or the simulator) through
        the relay: only use it on the simulator or with the robot suspended and the e-stop in hand.

    a3_relay_tool.py write-golden DIR
        Writes the fixed command / state frames the C++ unit test compares against.
"""

from __future__ import annotations

import argparse
import os
import struct
import sys
import time

DOF = 31
VERSION = 1
CMD_MAGIC = b"A3CM"
STATE_MAGIC = b"A3ST"
CMD_BYTES = 32 + 5 * DOF * 8
STATE_BYTES = 72 + 3 * DOF * 8 + 20 * 8

MODES = {0: "DISARMED", 1: "ACTIVE", 2: "HOLD", 3: "DAMP"}
REJECTS = {0: "none", 1: "bad_frame", 2: "bad_value", 3: "limit", 4: "seq", 5: "other_session", 6: "latched", 7: "arm_far",
           8: "peer", 9: "listen_only", 10: "no_state"}


def pack_command(session, seq, q_des, dq_des, tau_ff, kp, kd, flags=0, send_ns=0):
    for v in (q_des, dq_des, tau_ff, kp, kd):
        assert len(v) == DOF
    head = struct.pack("<4sHHQIIq", CMD_MAGIC, VERSION, DOF, session, seq, flags, send_ns)
    body = struct.pack(f"<{5 * DOF}d", *q_des, *dq_des, *tau_ff, *kp, *kd)
    out = head + body
    assert len(out) == CMD_BYTES
    return out


def unpack_command(buf):
    if len(buf) != CMD_BYTES:
        raise ValueError(f"command frame is {len(buf)} bytes, expected {CMD_BYTES}")
    magic, ver, dof, session, seq, flags, send_ns = struct.unpack_from("<4sHHQIIq", buf, 0)
    if magic != CMD_MAGIC or ver != VERSION or dof != DOF:
        raise ValueError("not an A3 relay command frame")
    arr = struct.unpack_from(f"<{5 * DOF}d", buf, 32)
    q, dq, tau, kp, kd = (list(arr[i * DOF:(i + 1) * DOF]) for i in range(5))
    return dict(session=session, seq=seq, flags=flags, send_ns=send_ns, q_des=q, dq_des=dq, tau_ff=tau, kp=kp, kd=kd)


def pack_state(relay_id, seq, mode, last_reject, flags, owner_session, last_accepted_seq, accepted_total, rejected_total,
               state_age_ns, data_age_ns, relay_ns, q, dq, tau, pelvis, torso):
    """pelvis / torso: 10 floats each (quat wxyz, gyro xyz, accel xyz)."""
    assert len(q) == len(dq) == len(tau) == DOF and len(pelvis) == len(torso) == 10
    head = struct.pack("<4sHHQIBBHQIIIIqqq", STATE_MAGIC, VERSION, DOF, relay_id, seq, mode, last_reject, flags, owner_session,
                       last_accepted_seq, accepted_total, rejected_total, 0, state_age_ns, data_age_ns, relay_ns)
    body = struct.pack(f"<{3 * DOF + 20}d", *q, *dq, *tau, *pelvis, *torso)
    out = head + body
    assert len(out) == STATE_BYTES
    return out


def unpack_state(buf):
    if len(buf) != STATE_BYTES:
        raise ValueError(f"state frame is {len(buf)} bytes, expected {STATE_BYTES}")
    (magic, ver, dof, relay_id, seq, mode, reject, flags, owner, last_seq, accepted, rejected, _res, state_age, data_age,
     relay_ns) = struct.unpack_from("<4sHHQIBBHQIIIIqqq", buf, 0)
    if magic != STATE_MAGIC or ver != VERSION or dof != DOF:
        raise ValueError("not an A3 relay state frame")
    arr = struct.unpack_from(f"<{3 * DOF + 20}d", buf, 72)
    return dict(relay_id=relay_id, seq=seq, mode=mode, last_reject=reject, flags=flags, owner_session=owner,
                last_accepted_seq=last_seq, accepted_total=accepted, rejected_total=rejected, state_age_ns=state_age,
                data_age_ns=data_age, relay_ns=relay_ns, q=list(arr[:DOF]), dq=list(arr[DOF:2 * DOF]),
                tau=list(arr[2 * DOF:3 * DOF]), pelvis=list(arr[3 * DOF:3 * DOF + 10]), torso=list(arr[3 * DOF + 10:]))


# ----- golden frames (fixed values; the C++ test builds the same ones) --------------------------------------------------------
def golden_command():
    q = [0.01 * i for i in range(DOF)]
    return pack_command(0x1122334455667788, 4242, q, [0.1 * i for i in range(DOF)], [-0.5 * i for i in range(DOF)],
                        [1.0 + i for i in range(DOF)], [0.25 * (i + 1) for i in range(DOF)], flags=7, send_ns=1_700_000_000_123_456_789)


def golden_state():
    return pack_state(0xAABBCCDDEEFF0011, 99, 2, 6, 0b111, 0x1122334455667788, 4242, 1000, 7, 12_345_678, 2_345_678,
                      1_700_000_000_987_654_321, [0.02 * i for i in range(DOF)], [0.03 * i for i in range(DOF)],
                      [0.04 * i for i in range(DOF)], [0.5, 0.5, 0.5, 0.5, 0.1, 0.2, 0.3, 1.0, 2.0, 9.81],
                      [1.0, 0.0, 0.0, 0.0, -0.1, -0.2, -0.3, -1.0, -2.0, 9.5])


# ----- tools ------------------------------------------------------------------------------------------------------------------
def _fmt_state(s):
    flags = s["flags"]
    return (f"mode={MODES.get(s['mode'], s['mode']):8s} valid={int(flags & 1)} imu2={int(flags >> 1 & 1)} fwd={int(flags >> 2 & 1)} "
            f"owner={s['owner_session']:016x} acc={s['accepted_total']} rej={s['rejected_total']}({REJECTS.get(s['last_reject'], '?')}) "
            f"age={s['state_age_ns'] / 1e6:6.2f}ms data_age={s['data_age_ns'] / 1e6:6.2f}ms q[0:3]="
            f"{[round(x, 3) for x in s['q'][:3]]} q[19:22]={[round(x, 3) for x in s['q'][19:22]]}")


def _sub_socket(ctx, host, port):
    import zmq
    sock = ctx.socket(zmq.SUB)
    sock.setsockopt(zmq.SUBSCRIBE, b"")
    sock.setsockopt(zmq.RCVHWM, 4)
    sock.setsockopt(zmq.CONFLATE, 1)
    sock.connect(f"tcp://{host}:{port}")
    return sock


def cmd_listen(a):
    import zmq
    ctx = zmq.Context()
    sock = _sub_socket(ctx, a.host, a.state_port)
    t_start = time.time()
    t_end = t_start + a.seconds
    last_print, n, bad = 0.0, 0, 0
    while time.time() < t_end:
        if not sock.poll(200):
            continue
        try:
            s = unpack_state(sock.recv(zmq.NOBLOCK))
        except ValueError as e:
            bad += 1
            print("bad frame:", e)
            continue
        n += 1
        if time.time() - last_print >= 0.5:
            last_print = time.time()
            # [n] is the number of state frames received so far (200 Hz), NOT milliseconds; t is wall-clock seconds since the start
            print(f"[{n:6d}] t={last_print - t_start:6.1f}s seq={s['seq']:8d} {_fmt_state(s)}")
    print(f"done: {n} state frames, {bad} bad")
    return 0 if n else 1


def cmd_fake_pc(a):
    import zmq
    ctx = zmq.Context()
    sub = _sub_socket(ctx, a.host, a.state_port)
    pub = ctx.socket(zmq.PUB)
    pub.setsockopt(zmq.SNDHWM, 4)
    pub.setsockopt(zmq.LINGER, 0)
    pub.connect(f"tcp://{a.host}:{a.command_port}")
    session = int.from_bytes(os.urandom(8), "little") | 1
    print(f"fake-pc: session {session:016x}; waiting for a valid state frame from {a.host}:{a.state_port} ...")
    s = None
    t0 = time.time()
    while time.time() - t0 < 10:
        if sub.poll(200):
            try:
                cand = unpack_state(sub.recv(zmq.NOBLOCK))
            except ValueError:
                continue
            if cand["flags"] & 1:
                s = cand
                break
    if s is None:
        print("fake-pc: no valid state frame within 10 s")
        return 1
    print("fake-pc: relay says", _fmt_state(s))
    if not s["flags"] & 4:
        print("fake-pc: the relay is in --listen-only mode (command forwarding off); frames will be refused")
    q = list(s["q"])
    kp, kd, zero = [a.kp] * DOF, [a.kd] * DOF, [0.0] * DOF
    time.sleep(0.3)  # let the PUB connection come up (slow joiner)
    period, seq = 1.0 / a.rate, 0
    t_hold_end = time.time() + a.hold_s
    t_end = t_hold_end + a.after_s
    next_t, last_print, last_mode = time.time(), 0.0, None
    while time.time() < t_end:
        now = time.time()
        if now < t_hold_end and now >= next_t:
            seq += 1
            pub.send(pack_command(session, seq, q, zero, zero, kp, kd, send_ns=time.time_ns()))
            next_t += period
        elif now >= t_hold_end and a.then == "garbage" and now >= next_t:
            pub.send(os.urandom(CMD_BYTES))
            next_t += period
        if sub.poll(5):
            try:
                st = unpack_state(sub.recv(zmq.NOBLOCK))
            except ValueError:
                continue
            if st["mode"] != last_mode or now - last_print >= 0.5:
                last_mode, last_print = st["mode"], now
                phase = "holding pose" if now < t_hold_end else f"{a.then}"
                print(f"t={now - t_hold_end + a.hold_s:5.2f}s [{phase:12s}] {_fmt_state(st)}")
    print(f"fake-pc: done (sent {seq} frames)")
    return 0


def cmd_write_golden(a):
    os.makedirs(a.dir, exist_ok=True)
    for name, data in (("a3_relay_golden_cmd.bin", golden_command()), ("a3_relay_golden_state.bin", golden_state())):
        with open(os.path.join(a.dir, name), "wb") as f:
            f.write(data)
        print("wrote", os.path.join(a.dir, name), len(data), "bytes")
    return 0


# SDK slot order of the 31-DOF state frame (include/robot_io/a3_layout_extra.hpp)
SLOT_NAMES = (["waist_yaw", "waist_roll", "waist_pitch", "neck_yaw", "neck_pitch"]
              + [f"{side}_{j}" for side in ("L", "R") for j in ("shoulder_p", "shoulder_r", "shoulder_y", "elbow", "wrist_r", "wrist_p", "wrist_y")]
              + [f"{side}_{j}" for side in ("L", "R") for j in ("hip_p", "hip_r", "hip_y", "knee", "ankle_p", "ankle_r")])


def cmd_snapshot(a):
    """Full-precision statistics of the state stream over --seconds: per slot mean / std / min / max of q, mean dq and tau, and the IMUs.

    Read-only (a SUB socket).  Use it to check that the numbers are alive (std > 0 for a robot that is not perfectly rigid), that the slot order
    is what the layout says, and what pose the robot is in before any command is armed."""
    import zmq
    ctx = zmq.Context()
    sock = _sub_socket(ctx, a.host, a.state_port)
    rows, t_end = [], time.time() + a.seconds
    while time.time() < t_end:
        if not sock.poll(200):
            continue
        try:
            rows.append(unpack_state(sock.recv(zmq.NOBLOCK)))
        except ValueError:
            pass
    if not rows:
        print("no state frames")
        return 1
    n = len(rows)
    seqs = [r["seq"] for r in rows]
    gaps = sum(1 for x, y in zip(seqs, seqs[1:]) if y != x + 1)
    mean = lambda xs: sum(xs) / len(xs)
    std = lambda xs: (sum((x - mean(xs)) ** 2 for x in xs) / len(xs)) ** 0.5
    print(f"{n} frames in {a.seconds:.0f} s ({n / a.seconds:.0f} Hz), seq gaps {gaps}, valid={sum(r['flags'] & 1 for r in rows)}/{n}, "
          f"mode(s)={sorted({MODES.get(r['mode'], r['mode']) for r in rows})}, state_age mean {mean([r['state_age_ns'] for r in rows]) / 1e6:.2f} ms max "
          f"{max(r['state_age_ns'] for r in rows) / 1e6:.2f} ms")
    print(f"{'slot':>3s} {'name':12s} {'q mean':>10s} {'q std':>10s} {'q min':>10s} {'q max':>10s} {'dq mean':>9s} {'tau mean':>9s}")
    for j in range(DOF):
        q = [r["q"][j] for r in rows]
        print(f"{j:3d} {SLOT_NAMES[j]:12s} {mean(q):10.5f} {std(q):10.2e} {min(q):10.5f} {max(q):10.5f} {mean([r['dq'][j] for r in rows]):9.4f} "
              f"{mean([r['tau'][j] for r in rows]):9.3f}")
    for name in ("pelvis", "torso"):
        v = [mean([r[name][k] for r in rows]) for k in range(10)]
        norm = sum(x * x for x in v[7:10]) ** 0.5
        print(f"{name:7s} quat wxyz {[round(x, 4) for x in v[:4]]} gyro {[round(x, 4) for x in v[4:7]]} accel {[round(x, 3) for x in v[7:10]]} |accel| {norm:.3f}")
    return 0


def cmd_record(a):
    """Full-rate, read-only recording of the relay's state stream into a .npz (see the module docstring)."""
    import numpy as np
    import zmq

    ctx = zmq.Context()
    sock = _sub_socket(ctx, a.host, a.state_port)
    cols = {k: [] for k in ("t_wall", "seq", "mode", "flags", "owner", "accepted", "rejected", "state_age_ns", "q", "dq", "tau", "pelvis", "torso")}

    def save():
        n = len(cols["seq"])
        if not n:
            return 0
        tmp = a.out + ".tmp.npz"
        np.savez(tmp, owner=np.array(cols["owner"], dtype=np.uint64), slot_names=np.array(SLOT_NAMES),
                 **{k: np.array(v) for k, v in cols.items() if k != "owner"})
        os.replace(tmp, a.out)
        return n

    t0 = time.time()
    t_end = t0 + a.seconds
    last_save = last_print = t0
    bad = 0
    try:
        while time.time() < t_end:
            if not sock.poll(200):
                continue
            try:
                s = unpack_state(sock.recv(zmq.NOBLOCK))
            except ValueError:
                bad += 1
                continue
            cols["t_wall"].append(time.time())
            for k, key in (("seq", "seq"), ("mode", "mode"), ("flags", "flags"), ("owner", "owner_session"), ("accepted", "accepted_total"),
                           ("rejected", "rejected_total"), ("state_age_ns", "state_age_ns"), ("q", "q"), ("dq", "dq"), ("tau", "tau"),
                           ("pelvis", "pelvis"), ("torso", "torso")):
                cols[k].append(s[key])
            now = time.time()
            if now - last_print >= 5.0:
                last_print = now
                print(f"t={now - t0:6.1f}s frames={len(cols['seq'])} {_fmt_state(s)}")
            if now - last_save >= 30.0:
                last_save = now
                save()
    except KeyboardInterrupt:
        pass
    n = save()
    print(f"saved {n} frames ({bad} bad) to {a.out}" if n else "no state frames received: nothing saved")
    return 0 if n else 1


def main(argv=None):
    try:
        sys.stdout.reconfigure(line_buffering=True)  # `listen | tee file`: a pipe is block-buffered by default and Ctrl-C would lose everything
    except (AttributeError, ValueError):
        pass
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    for name in ("listen", "snapshot", "record", "fake-pc"):
        p = sub.add_parser(name)
        p.add_argument("--host", required=True)
        p.add_argument("--state-port", type=int, default=5570)
        p.add_argument("--command-port", type=int, default=5571)
        if name in ("listen", "snapshot", "record"):
            p.add_argument("--seconds", type=float, default=400.0 if name == "record" else 10.0)
        if name == "record":
            p.add_argument("--out", required=True, help="output .npz")
        else:
            p.add_argument("--rate", type=float, default=50.0)
            p.add_argument("--hold-s", type=float, default=3.0)
            p.add_argument("--after-s", type=float, default=2.0, help="how long to keep watching after the frames stop")
            p.add_argument("--then", choices=("silence", "garbage"), default="silence")
            p.add_argument("--kp", type=float, default=20.0)
            p.add_argument("--kd", type=float, default=1.0)
    g = sub.add_parser("write-golden")
    g.add_argument("dir")
    a = ap.parse_args(argv)
    return {"listen": cmd_listen, "snapshot": cmd_snapshot, "record": cmd_record, "fake-pc": cmd_fake_pc, "write-golden": cmd_write_golden}[a.cmd](a)


if __name__ == "__main__":
    sys.exit(main())
