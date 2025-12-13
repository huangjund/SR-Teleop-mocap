"""Simple UDP listener to inspect teleop UDP payloads.

Run alongside the C++ teleop sender to confirm that the streamed wrist
poses and hand joint angles look correct. The script binds to the UDP
port used by ``teleop_main`` and prints the decoded contents of each
packet. It also renders a simple 3D visualization of the left and right
wrist frames so you can see trajectories update live.

Example:
    python scripts/test_udp_listener.py --listen-ip 0.0.0.0 --listen-port 16000
"""

from __future__ import annotations

import argparse
import select
import socket
import sys
from typing import Dict, Sequence

import pinocchio as pin

try:
    import matplotlib.pyplot as plt
except ModuleNotFoundError as exc:
    raise ModuleNotFoundError(
        "matplotlib is required for real-time wrist visualization; please install it first"
    ) from exc


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
    parser.add_argument(
        "--no-visualize",
        action="store_true",
        help="Disable the live wrist visualization window",
    )
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


class WristVisualizer:
    """Live 3D visualization of wrist positions."""

    def __init__(self) -> None:
        plt.ion()

        self.fig = plt.figure("Teleop wrist poses")
        self.ax = self.fig.add_subplot(111, projection="3d")
        self.ax.set_xlabel("X (m)")
        self.ax.set_ylabel("Y (m)")
        self.ax.set_zlabel("Z (m)")
        self.ax.view_init(elev=25, azim=-65)
        self.ax.set_box_aspect([1, 1, 1])

        self.scatter = {}
        self.traces = {}
        self.history: Dict[str, list[tuple[float, float, float]]] = {}

        for side, color in {"left": "tab:blue", "right": "tab:orange"}.items():
            self.history[side] = []
            self.scatter[side] = self.ax.scatter([], [], [], color=color, label=f"{side.title()} wrist")
            (self.traces[side],) = self.ax.plot([], [], [], color=color, alpha=0.5, linewidth=1.0)

        self.frame_text = self.ax.text2D(0.02, 0.98, "Frame ?", transform=self.ax.transAxes)
        self.ax.legend(loc="upper right")

    def update(self, wrist_poses: Dict[str, list[float]], frame_idx: int | None) -> None:
        self.frame_text.set_text(f"Frame {frame_idx}" if frame_idx is not None else "Frame ?")

        for side, pose in wrist_poses.items():
            position = tuple(pose[:3])
            history = self.history.setdefault(side, [])
            history.append(position)
            if len(history) > 200:
                history.pop(0)

            xs, ys, zs = zip(*history)
            self.scatter.setdefault(side, self.ax.scatter([], [], []))
            self.traces.setdefault(side, self.ax.plot([], [], [])[0])

            self.scatter[side]._offsets3d = ([position[0]], [position[1]], [position[2]])
            self.traces[side].set_data(xs, ys)
            self.traces[side].set_3d_properties(zs)

        self.ax.relim()
        self.ax.autoscale_view()
        plt.pause(0.001)


def listen(args: argparse.Namespace) -> None:
    udp_sock = _bind_socket(args.listen_ip, args.listen_port)

    visualizer = None if args.no_visualize else WristVisualizer()

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

                if hand_targets and not args.no_hand:
                    print("  Hand joints:")
                    for side, angles in sorted(hand_targets.items()):
                        print(f"    {side}: {_format_angles(angles)}")

                if not wrist_poses and not hand_targets:
                    print("  (No wrist or hand lines decoded)")

                if visualizer and wrist_poses:
                    visualizer.update(wrist_poses, frame_idx)
    except KeyboardInterrupt:
        print("\nStopping UDP listener.")
    finally:
        udp_sock.close()


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv or sys.argv[1:])
    listen(args)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
