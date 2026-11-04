#!/usr/bin/env python3
"""Left Revo2 three-finger pinch reciprocating test.

Cycle:
  close (3-finger pinch) -> hold closed_s -> open -> hold open_s -> repeat

Default: left hand slave_id=0x7E (126), port auto among ttyCH343USB* / ACM*.
Ctrl+C stops and leaves the hand open.

Examples (on Orin):
  python3 gear_sonic_deploy/scripts/g1_revo2_left_pinch3_cycle.py
  python3 gear_sonic_deploy/scripts/g1_revo2_left_pinch3_cycle.py --port /dev/ttyCH343USB0 --cycles 5
"""
from __future__ import annotations

import argparse
import asyncio
import sys
import time

import bc_stark_sdk.main_mod as libstark


# Official unified range: 0=open, 1000=closed
# 3-finger pinch: thumb + index + middle; ring/pinky stay open.
# ThumbAux slightly closed for opposition (path similar to built-in pinch-three).
PINCH3_POS = [700, 700, 1000, 1000, 0, 0]  # Thumb, ThumbAux, Index, Middle, Ring, Pinky
OPEN_POS = [0, 0, 0, 0, 0, 0]
MOVE_MS = [800, 800, 800, 800, 400, 400]


async def _await_io(coro, label: str, timeout_s: float = 1.0) -> bool:
    """SDK often logs Modbus write timeout even when the hand still moves.

    Never block forever on native futures — cancel after timeout and continue.
    """
    try:
        await asyncio.wait_for(coro, timeout=timeout_s)
        return True
    except asyncio.TimeoutError:
        print("[revo2] %s: no ACK within %.1fs (continuing)" % (label, timeout_s))
        return False
    except Exception as e:
        print("[revo2] %s: %s: %s (continuing)" % (label, type(e).__name__, e))
        return False


async def open_left_client(port: str, baud: libstark.Baudrate, slave_id: int):
    client = await libstark.modbus_open(port, baud)
    # Skip set_hardware_type here: on busy hands it hits reg 1098 ServerDeviceBusy
    # and can interfere with subsequent gesture/position writes.
    # Require a real device ACK before claiming the port is usable.
    ok = False
    last = None
    for getter in ("get_device_info", "get_finger_positions", "get_motor_status"):
        if not hasattr(client, getter):
            continue
        try:
            r = await asyncio.wait_for(getattr(client, getter)(slave_id), timeout=1.2)
            print("[revo2] probe %s -> %s" % (getter, r))
            ok = True
            break
        except Exception as e:
            last = e
    if not ok:
        libstark.modbus_close(client)
        raise RuntimeError(
            "no Modbus ACK on %s slave=%s (%s). "
            "Check wrist DIP=Modbus-RTU (not CANFD) and 485 wiring."
            % (port, slave_id, last)
        )
    return client


async def move_positions(client, slave_id: int, positions, durations=None):
    durations = durations or MOVE_MS
    if hasattr(client, "set_finger_positions_and_durations"):
        await _await_io(
            client.set_finger_positions_and_durations(slave_id, positions, durations),
            "set_pos+dur",
            1.0,
        )
    else:
        await _await_io(
            client.set_finger_positions(slave_id, positions),
            "set_pos",
            1.0,
        )


async def run_builtin(client, slave_id: int, action_id):
    await _await_io(client.run_action_sequence(slave_id, action_id), "builtin", 1.5)


async def one_cycle(client, slave_id: int, closed_s: float, open_s: float, use_builtin: bool):
    if use_builtin:
        print("[revo2] CLOSE DefaultGesturePinchThree")
        await run_builtin(client, slave_id, libstark.ActionSequenceId.DefaultGesturePinchThree)
        # built-in motion time ~1s; still enforce hold after
        await asyncio.sleep(1.0)
    else:
        print("[revo2] CLOSE pinch3 positions=%s" % (PINCH3_POS,))
        await move_positions(client, slave_id, PINCH3_POS)
        await asyncio.sleep(max(d for d in MOVE_MS) / 1000.0)

    print("[revo2] HOLD closed %.1fs" % closed_s)
    await asyncio.sleep(closed_s)

    if use_builtin:
        print("[revo2] OPEN DefaultGestureOpen")
        await run_builtin(client, slave_id, libstark.ActionSequenceId.DefaultGestureOpen)
        await asyncio.sleep(1.0)
    else:
        print("[revo2] OPEN positions=%s" % (OPEN_POS,))
        await move_positions(client, slave_id, OPEN_POS)
        await asyncio.sleep(max(d for d in MOVE_MS) / 1000.0)

    print("[revo2] HOLD open %.1fs" % open_s)
    await asyncio.sleep(open_s)


async def main_async(args):
    # Prefer CH343 USB-485 (official path); fall back to older ACM Dual Serial.
    ports = [args.port] if args.port else [
        "/dev/ttyCH343USB0",
        "/dev/ttyCH343USB1",
        "/dev/ttyCH343USB2",
        "/dev/ttyACM0",
        "/dev/ttyACM1",
    ]
    bauds = [libstark.Baudrate.Baud460800, libstark.Baudrate.Baud115200]
    client = None
    used = None
    last_err = None
    for port in ports:
        for baud in bauds:
            try:
                print("[revo2] try %s %s slave=%d" % (port, baud, args.slave_id))
                client = await open_left_client(port, baud, args.slave_id)
                used = (port, baud)
                break
            except Exception as e:
                last_err = e
                print("[revo2]  skip: %s" % e)
        if client is not None:
            break
    if client is None:
        raise RuntimeError("failed to open left Revo2 (%s)" % last_err)

    print("[revo2] connected %s slave=%d  cycles=%s  closed=%.1fs open=%.1fs builtin=%s"
          % (used, args.slave_id, args.cycles, args.closed_s, args.open_s, args.builtin))

    n = 0
    try:
        while args.cycles <= 0 or n < args.cycles:
            n += 1
            print("======== cycle %d ========" % n)
            await one_cycle(client, args.slave_id, args.closed_s, args.open_s, args.builtin)
    except asyncio.CancelledError:
        pass
    finally:
        try:
            print("[revo2] finishing: open hand")
            await move_positions(client, args.slave_id, OPEN_POS)
            await asyncio.sleep(0.8)
        except Exception as e:
            print("[revo2] open-on-exit failed:", e)
        libstark.modbus_close(client)
        print("[revo2] closed client")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=None, help="e.g. /dev/ttyACM0 (default: try ACM0 then ACM1)")
    ap.add_argument("--slave-id", type=int, default=0x7E, help="left hand default 0x7E=126")
    ap.add_argument("--closed-s", type=float, default=3.0)
    ap.add_argument("--open-s", type=float, default=1.0)
    ap.add_argument("--cycles", type=int, default=0, help="0 = infinite until Ctrl+C")
    ap.add_argument(
        "--builtin",
        action="store_true",
        help="use built-in DefaultGesturePinchThree / Open (else explicit 3-finger positions)",
    )
    ap.add_argument(
        "--positions",
        action="store_true",
        default=True,
        help="use explicit pinch3 positions (default)",
    )
    args = ap.parse_args()
    # default: positions mode unless --builtin
    if args.builtin:
        pass
    try:
        asyncio.run(main_async(args))
    except KeyboardInterrupt:
        print("\n[revo2] interrupted")
        return 0
    return 0


if __name__ == "__main__":
    sys.exit(main())
