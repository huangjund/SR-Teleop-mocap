"""Interactive slider that fakes mocap teleop UDP output for dual arms.

This utility reproduces the UDP packet format emitted by ``mocap_teleop`` so
you can drive the PyBullet visualizer (``scripts/udp_wrist_to_ik.py``) without
Axis Studio or the C++ teleop binary. A lightweight Tkinter UI exposes sliders
for joint angles and streams packets continuously to the configured UDP
endpoint.
"""

from __future__ import annotations

import argparse
import socket
import sys
import time
import tkinter as tk
from typing import Dict, Iterable, Sequence


def parse_args(argv: Sequence[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Send fake teleop UDP joint packets with a slider UI")
    parser.add_argument("--dest-ip", default="127.0.0.1", help="Destination IP for the UDP joint stream")
    parser.add_argument("--dest-port", type=int, default=15000, help="Destination UDP port for arm joints")
    parser.add_argument(
        "--rate-hz",
        type=float,
        default=30.0,
        help="Streaming rate in Hz (controls how often slider values are broadcast)",
    )
    parser.add_argument(
        "--sides", nargs="+", choices=["left", "right"], default=["left", "right"], help="Arms to stream"
    )
    parser.add_argument(
        "--joint-limit",
        type=float,
        default=180,
        help="Symmetric slider limit in radians for each joint (default: +/-pi)",
    )
    return parser.parse_args(argv)


def _create_sliders(root: tk.Tk, sides: Iterable[str], *, joint_limit: float) -> Dict[str, list[tk.Scale]]:
    sliders: Dict[str, list[tk.Scale]] = {}

    for side in sides:
        frame = tk.LabelFrame(root, text=f"{side.capitalize()} arm")
        frame.pack(fill=tk.BOTH, expand=True, padx=8, pady=4)

        sliders[side] = []
        for idx in range(6):
            label = tk.Label(frame, text=f"Joint {idx + 1}")
            label.grid(row=idx, column=0, sticky=tk.W, padx=6, pady=2)

            scale = tk.Scale(
                frame,
                from_=-joint_limit,
                to=joint_limit,
                resolution=0.01,
                orient=tk.HORIZONTAL,
                length=280,
            )
            scale.grid(row=idx, column=1, sticky=tk.EW, padx=6, pady=2)
            sliders[side].append(scale)

        frame.columnconfigure(1, weight=1)

    return sliders


def _gather_angles(sliders: Dict[str, list[tk.Scale]]) -> Dict[str, list[float]]:
    angles: Dict[str, list[float]] = {}
    for side, params in sliders.items():
        angles[side] = [float(scale.get()) for scale in params]
    return angles


def stream_fake_packets(args: argparse.Namespace) -> None:
    root = tk.Tk()
    root.title("Fake teleop slider")

    sliders = _create_sliders(root, args.sides, joint_limit=args.joint_limit)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    destination = (args.dest_ip, args.dest_port)

    print(
        "Streaming fake teleop packets to "
        f"{args.dest_ip}:{args.dest_port} for sides: {', '.join(args.sides)}. "
        "Close the window or press Ctrl+C to stop."
    )

    running = True
    frame_idx = 0
    period_ms = int(1000.0 / args.rate_hz) if args.rate_hz > 0 else 0

    def on_close() -> None:
        nonlocal running
        running = False
        root.destroy()

    def send_packet() -> None:
        nonlocal frame_idx
        if not running:
            return

        angles = _gather_angles(sliders)
        lines = [f"frame,{frame_idx}"]
        for side in args.sides:
            side_angles = angles.get(side, [])
            formatted = ",".join(f"{angle:.6f}" for angle in side_angles)
            lines.append(f"joint,{side},{formatted}")

        payload = "\n".join(lines)
        sock.sendto(payload.encode("utf-8"), destination)

        frame_idx += 1
        if period_ms > 0:
            root.after(period_ms, send_packet)
        else:
            root.after_idle(send_packet)

    root.protocol("WM_DELETE_WINDOW", on_close)
    root.after(0, send_packet)

    try:
        root.mainloop()
    except KeyboardInterrupt:
        print("Stopping fake teleop sender.")
        on_close()


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv or sys.argv[1:])
    stream_fake_packets(args)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
