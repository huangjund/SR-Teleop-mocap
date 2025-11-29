"""Interactive slider that fakes mocap teleop UDP output for dual arms.

This utility reproduces the UDP packet format emitted by ``mocap_teleop`` so
you can drive the PyBullet visualizer (``scripts/udp_wrist_to_ik.py``) without
Axis Studio or the C++ teleop binary. Use the PyBullet debug sliders to update
joint angles in real time; packets are streamed continuously to the configured
UDP endpoint.
"""

from __future__ import annotations

import argparse
import math
import socket
import sys
import time
from typing import Dict, Iterable, Sequence

import pybullet as p


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
    parser.add_argument("--sides", nargs="+", choices=["left", "right"], default=["left", "right"], help="Arms to stream")
    parser.add_argument(
        "--joint-limit",
        type=float,
        default=math.pi,
        help="Symmetric slider limit in radians for each joint (default: +/-pi)",
    )
    return parser.parse_args(argv)


def _create_sliders(sides: Iterable[str], *, joint_limit: float) -> Dict[str, list[int]]:
    slider_ids: Dict[str, list[int]] = {}
    for side in sides:
        slider_ids[side] = []
        for idx in range(6):
            slider_ids[side].append(
                p.addUserDebugParameter(
                    f"{side} joint {idx + 1}",
                    -joint_limit,
                    joint_limit,
                    0.0,
                )
            )
    return slider_ids


def _gather_angles(slider_ids: Dict[str, list[int]]) -> Dict[str, list[float]]:
    angles: Dict[str, list[float]] = {}
    for side, params in slider_ids.items():
        angles[side] = [float(p.readUserDebugParameter(param_id)) for param_id in params]
    return angles


def stream_fake_packets(args: argparse.Namespace) -> None:
    client = p.connect(p.GUI)
    try:
        p.resetDebugVisualizerCamera(cameraDistance=2.5, cameraYaw=50, cameraPitch=-30, cameraTargetPosition=[0, 0, 0])
        # Keep the PyBullet GUI controls enabled so the debug sliders remain visible.
        # Disabling the GUI hides the slider pane, which led to a blank window for some
        # users when running this tool alongside ``udp_wrist_to_ik.py``.
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)
        slider_ids = _create_sliders(args.sides, joint_limit=args.joint_limit)

        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        destination = (args.dest_ip, args.dest_port)

        print(
            "Streaming fake teleop packets to "
            f"{args.dest_ip}:{args.dest_port} for sides: {', '.join(args.sides)}. "
            "Close the PyBullet window or press Ctrl+C to stop."
        )

        frame_idx = 0
        period = 1.0 / args.rate_hz if args.rate_hz > 0 else 0.0
        while True:
            angles = _gather_angles(slider_ids)
            lines = [f"frame,{frame_idx}"]
            for side in args.sides:
                side_angles = angles.get(side, [])
                formatted = ",".join(f"{angle:.6f}" for angle in side_angles)
                lines.append(f"joint,{side},{formatted}")

            payload = "\n".join(lines)
            sock.sendto(payload.encode("utf-8"), destination)

            frame_idx += 1
            if period > 0:
                time.sleep(period)
    except KeyboardInterrupt:
        print("Stopping fake teleop sender.")
    finally:
        p.disconnect(physicsClientId=client)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv or sys.argv[1:])
    stream_fake_packets(args)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
