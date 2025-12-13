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
import queue
import select
import socket
import sys
import threading
import time
from collections import deque
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
    """Live 3D visualization of wrist positions in a background thread.

    The visualizer runs its own matplotlib event loop so network packet
    handling stays responsive. Only the newest pose is rendered to avoid
    falling behind when packets arrive quickly.
    """

    def __init__(self, history_len: int = 400, idle_refresh_s: float = 0.05) -> None:
        self.history_len = history_len
        self.idle_refresh_s = idle_refresh_s

        self._queue: "queue.SimpleQueue[tuple[int | None, Dict[str, list[float]]]]" = queue.SimpleQueue()
        self._history: Dict[str, deque[tuple[float, float, float]]] = {
            "left": deque(maxlen=history_len),
            "right": deque(maxlen=history_len),
        }
        self._running = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)

    def start(self) -> None:
        if self._running.is_set():
            return
        self._running.set()
        self._thread.start()

    def stop(self) -> None:
        if not self._running.is_set():
            return
        self._running.clear()
        self._thread.join(timeout=1.0)

    def submit(self, wrist_poses: Dict[str, list[float]], frame_idx: int | None) -> None:
        """Send the newest wrist poses to the renderer without blocking UDP I/O."""

        if not self._running.is_set():
            return

        # Drop stale points if the UI lags to keep rendering real time.
        try:
            while self._queue.qsize() > 3:
                self._queue.get_nowait()
        except queue.Empty:
            pass

        self._queue.put((frame_idx, wrist_poses))

    def _run(self) -> None:
        plt.ion()

        fig = plt.figure("Teleop wrist poses (live)")
        ax = fig.add_subplot(111, projection="3d")
        ax.set_xlabel("X (m)")
        ax.set_ylabel("Y (m)")
        ax.set_zlabel("Z (m)")
        ax.view_init(elev=25, azim=-65)
        ax.set_box_aspect([1, 1, 1])

        scatter = {
            "left": ax.scatter([], [], [], color="tab:blue", label="Left wrist"),
            "right": ax.scatter([], [], [], color="tab:orange", label="Right wrist"),
        }
        traces = {
            "left": ax.plot([], [], [], color="tab:blue", alpha=0.6, linewidth=1.0)[0],
            "right": ax.plot([], [], [], color="tab:orange", alpha=0.6, linewidth=1.0)[0],
        }

        frame_text = ax.text2D(0.02, 0.98, "Frame ?", transform=ax.transAxes)
        ax.legend(loc="upper right")

        last_update = time.time()

        while self._running.is_set():
            try:
                frame_idx, wrist_poses = self._queue.get(timeout=self.idle_refresh_s)
                while not self._queue.empty():
                    frame_idx, wrist_poses = self._queue.get_nowait()
            except queue.Empty:
                frame_idx = None
                wrist_poses = {}

            if wrist_poses:
                frame_text.set_text(f"Frame {frame_idx}" if frame_idx is not None else "Frame ?")

                for side, pose in wrist_poses.items():
                    position = tuple(pose[:3])
                    self._history[side].append(position)

                    xs, ys, zs = zip(*self._history[side])
                    scatter[side]._offsets3d = ([position[0]], [position[1]], [position[2]])
                    traces[side].set_data(xs, ys)
                    traces[side].set_3d_properties(zs)

                all_positions = [p for hist in self._history.values() for p in hist]
                if all_positions:
                    xs, ys, zs = zip(*all_positions)
                    margin = 0.05
                    ax.set_xlim(min(xs) - margin, max(xs) + margin)
                    ax.set_ylim(min(ys) - margin, max(ys) + margin)
                    ax.set_zlim(min(zs) - margin, max(zs) + margin)

                last_update = time.time()

            # Keep the UI responsive even when idle.
            fig.canvas.draw_idle()
            plt.pause(0.001)

            # Avoid tight loop if nothing is arriving.
            if time.time() - last_update > 1.0:
                time.sleep(self.idle_refresh_s)

        plt.close(fig)


def listen(args: argparse.Namespace) -> None:
    udp_sock = _bind_socket(args.listen_ip, args.listen_port)

    visualizer = None if args.no_visualize else WristVisualizer()
    if visualizer:
        visualizer.start()

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
                    visualizer.submit(wrist_poses, frame_idx)
    except KeyboardInterrupt:
        print("\nStopping UDP listener.")
    finally:
        if visualizer:
            visualizer.stop()
        udp_sock.close()


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv or sys.argv[1:])
    listen(args)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
