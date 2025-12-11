"""Simple UDP listener to inspect teleop UDP payloads.

Run alongside the C++ teleop sender to confirm that the streamed wrist
poses and hand joint angles look correct. The script binds to the UDP
port used by ``teleop_main`` and prints the decoded contents of each
packet.

Example:
    python scripts/test_udp_listener.py --listen-ip 0.0.0.0 --listen-port 16000
"""

from __future__ import annotations

import argparse
import select
import socket
import sys
from typing import Dict, Sequence

import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

import pinocchio as pin


def _parse_packet(packet: bytes) -> tuple[int | None, Dict[str, list[float]], Dict[str, list[float]]]:
    """Decode a UDP datagram carrying wrist poses and hand joint angles.

    Returns the frame index (if present) plus per-side wrist pose and hand angle lists.
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

    wrist_poses: Dict[str, list[float]] = {}
    hand_targets: Dict[str, list[float]] = {}
    for line in lines[1:]:
        parts = [part.strip() for part in line.split(",")]
        if len(parts) < 3:
            continue

        prefix, side = parts[0].lower(), parts[1].lower()
        try:
            values = [float(value) for value in parts[2:]]
        except ValueError:
            continue

        if prefix == "wrist" and len(values) == 7:
            wrist_poses[side] = values
        elif prefix == "hand":
            hand_targets[side] = values

    return frame_idx, wrist_poses, hand_targets


def _bind_socket(ip: str, port: int) -> socket.socket:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((ip, port))
    sock.setblocking(False)
    return sock


def parse_args(argv: Sequence[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Listen for teleop wrist/hand payloads over UDP")
    parser.add_argument("--listen-ip", default="0.0.0.0", help="IP address to bind the UDP listeners")
    parser.add_argument(
        "--listen-port",
        type=int,
        default=16000,
        help="UDP port carrying wrist poses and dexterous-hand joint angles",
    )
    parser.add_argument("--no-hand", action="store_true", help="Ignore dexterous-hand UDP packets")
    return parser.parse_args(argv)


def _format_angles(angles: list[float]) -> str:
    return ", ".join(f"{angle:.3f}" for angle in angles)


def _quat_to_euler(quaternion: Sequence[float]) -> tuple[float, float, float]:
    """Convert a quaternion (x, y, z, w) to Euler roll, pitch, yaw (radians)."""

    x, y, z, w = quaternion
    eigen_quat = pin.Quaternion(w, x, y, z)
    eigen_quat.normalize()
    roll, pitch, yaw = pin.rpy.matrixToRpy(eigen_quat.matrix())

    return float(roll), float(pitch), float(yaw)


def _setup_plot():
    plt.ion()
    fig = plt.figure(figsize=(7, 7))
    ax = fig.add_subplot(111, projection="3d")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")
    ax.scatter([0], [0], [0], marker="^", color="black", label="base")

    line_map = {}
    for side, color in (("left", "tab:blue"), ("right", "tab:orange")):
        (line,) = ax.plot([], [], [], marker="o", linestyle="-", color=color, label=f"{side} wrist")
        line_map[side] = line

    ax.legend(loc="upper left")
    fig.tight_layout()
    return fig, ax, line_map


def _adjust_axes(ax, points: list[tuple[float, float, float]]) -> None:
    if not points:
        return

    xs, ys, zs = zip(*points)
    xs += (0.0,)
    ys += (0.0,)
    zs += (0.0,)

    x_range = max(xs) - min(xs)
    y_range = max(ys) - min(ys)
    z_range = max(zs) - min(zs)
    max_range = max(x_range, y_range, z_range, 0.1)

    center_x = (max(xs) + min(xs)) / 2
    center_y = (max(ys) + min(ys)) / 2
    center_z = (max(zs) + min(zs)) / 2

    pad = max_range / 2
    ax.set_xlim(center_x - pad, center_x + pad)
    ax.set_ylim(center_y - pad, center_y + pad)
    ax.set_zlim(center_z - pad, center_z + pad)


def _update_plot(ax, line_map: Dict[str, Line2D], wrist_traces: Dict[str, list[tuple[float, float, float]]]) -> None:
    all_points: list[tuple[float, float, float]] = []
    for side, line in line_map.items():
        points = wrist_traces.get(side, [])
        if points:
            xs, ys, zs = zip(*points)
            line.set_data(xs, ys)
            line.set_3d_properties(zs)
            all_points.extend(points)
        else:
            line.set_data([], [])
            line.set_3d_properties([])

    _adjust_axes(ax, all_points)
    ax.figure.canvas.draw()
    ax.figure.canvas.flush_events()


def listen(args: argparse.Namespace) -> None:
    udp_sock = _bind_socket(args.listen_ip, args.listen_port)
    fig, ax, line_map = _setup_plot()
    wrist_traces: Dict[str, list[tuple[float, float, float]]] = {}

    print(f"Listening for wrist/hand UDP on {args.listen_ip}:{args.listen_port}")
    print("Press Ctrl+C to exit.\n")

    try:
        while True:
            readable, _, _ = select.select([udp_sock], [], [], 1.0)
            for sock in readable:
                try:
                    packet, addr = sock.recvfrom(4096)
                except OSError:
                    continue

                frame_idx, wrist_poses, hand_targets = _parse_packet(packet)
                frame_label = f"Frame {frame_idx}" if frame_idx is not None else "Frame ?"
                print(f"[{frame_label}] Received packet from {addr[0]}:{addr[1]}")

                if wrist_poses:
                    print("  Wrist poses (pos_x, pos_y, pos_z, roll, pitch, yaw):")
                    for side, values in sorted(wrist_poses.items()):
                        pos = values[:3]
                        quat = values[3:]
                        euler = _quat_to_euler(quat)
                        print(f"    {side}: {_format_angles([*pos, *euler])}")
                        wrist_traces.setdefault(side, []).append(tuple(pos))

                    _update_plot(ax, line_map, wrist_traces)

                if hand_targets and not args.no_hand:
                    print("  Hand joints:")
                    for side, angles in sorted(hand_targets.items()):
                        print(f"    {side}: {_format_angles(angles)}")

                if not wrist_poses and not hand_targets:
                    print("  (No wrist or hand lines decoded)")
    except KeyboardInterrupt:
        print("\nStopping UDP listener.")
    finally:
        udp_sock.close()
        plt.close(fig)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv or sys.argv[1:])
    listen(args)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
