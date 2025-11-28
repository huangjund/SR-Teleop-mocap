"""Simple UDP listener to inspect teleop joint-angle packets.

Run alongside the C++ teleop sender to confirm that the streamed joint
angles look correct. The script binds to the same UDP ports used by the
Python PyBullet visualizer and prints the decoded contents of each packet.

Example:
    python scripts/test_udp_listener.py --listen-ip 0.0.0.0 \
        --listen-port 15000 --listen-hand-port 15001
"""

from __future__ import annotations

import argparse
import select
import socket
import sys
from typing import Dict, Sequence


def _parse_packet(packet: bytes) -> tuple[int | None, Dict[str, list[float]], Dict[str, list[float]]]:
    """Decode a UDP datagram carrying arm/hand joint angles.

    Returns the frame index (if present) plus per-side arm and hand angle lists.
    """

    decoded = packet.decode("utf-8", errors="ignore")
    lines = [line.strip() for line in decoded.splitlines() if line.strip()]
    if not lines:
        return None, {}, {}

    frame_idx: int | None = None
    header = lines[0].split(",")
    if len(header) >= 2 and header[0].strip().lower() == "frame":
        try:
            frame_idx = int(header[1])
        except ValueError:
            frame_idx = None

    arm_targets: Dict[str, list[float]] = {}
    hand_targets: Dict[str, list[float]] = {}
    for line in lines[1:]:
        parts = [part.strip() for part in line.split(",")]
        if len(parts) < 3:
            continue

        prefix, side = parts[0].lower(), parts[1].lower()
        try:
            angles = [float(value) for value in parts[2:]]
        except ValueError:
            continue

        if prefix == "joint":
            arm_targets[side] = angles
        elif prefix == "hand":
            hand_targets[side] = angles

    return frame_idx, arm_targets, hand_targets


def _bind_socket(ip: str, port: int) -> socket.socket:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((ip, port))
    sock.setblocking(False)
    return sock


def parse_args(argv: Sequence[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Listen for teleop joint angles over UDP")
    parser.add_argument("--listen-ip", default="0.0.0.0", help="IP address to bind the UDP listeners")
    parser.add_argument("--listen-port", type=int, default=15000, help="UDP port carrying arm joint angles")
    parser.add_argument(
        "--listen-hand-port",
        type=int,
        default=15001,
        help="UDP port carrying dexterous-hand joint angles",
    )
    parser.add_argument("--no-hand", action="store_true", help="Ignore dexterous-hand UDP packets")
    return parser.parse_args(argv)


def _format_angles(angles: list[float]) -> str:
    return ", ".join(f"{angle:.3f}" for angle in angles)


def listen(args: argparse.Namespace) -> None:
    arm_sock = _bind_socket(args.listen_ip, args.listen_port)
    hand_sock = None if args.no_hand else _bind_socket(args.listen_ip, args.listen_hand_port)

    print(
        f"Listening for arm joints on {args.listen_ip}:{args.listen_port}"
        + ("" if args.no_hand else f" and hands on {args.listen_ip}:{args.listen_hand_port}")
    )
    print("Press Ctrl+C to exit.\n")

    sockets = [arm_sock] + ([hand_sock] if hand_sock is not None else [])

    try:
        while True:
            readable, _, _ = select.select(sockets, [], [], 1.0)
            for sock in readable:
                try:
                    packet, addr = sock.recvfrom(4096)
                except OSError:
                    continue

                frame_idx, arm_targets, hand_targets = _parse_packet(packet)
                frame_label = f"Frame {frame_idx}" if frame_idx is not None else "Frame ?"
                stream_label = "hand" if sock is hand_sock else "arm"
                print(f"[{frame_label}] Received {stream_label} packet from {addr[0]}:{addr[1]}")

                if arm_targets:
                    print("  Arm joints:")
                    for side, angles in sorted(arm_targets.items()):
                        print(f"    {side}: {_format_angles(angles)}")

                if hand_targets:
                    print("  Hand joints:")
                    for side, angles in sorted(hand_targets.items()):
                        print(f"    {side}: {_format_angles(angles)}")

                if not arm_targets and not hand_targets:
                    print("  (No joint lines decoded)")
    except KeyboardInterrupt:
        print("\nStopping UDP listener.")
    finally:
        arm_sock.close()
        if hand_sock is not None:
            hand_sock.close()


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv or sys.argv[1:])
    listen(args)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
