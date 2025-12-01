"""Convert streamed wrist poses into Fanuc arm/hand joint commands with calibration.

This helper listens for UDP packets carrying 6D wrist poses (plus optional hand
joint angles), runs Pinocchio IK to obtain Fanuc arm joints, and re-broadcasts
those joints over UDP. It mirrors the IK strategy from
``scripts/fake_cartesian_teleop_slider.py`` while adding a one-time calibration
step to align the mocap/device frame with the robot base frame.

Keyboard controls (matching the C++ teleop pipeline):
- Hold ``k`` to stream live IK solutions to the output UDP endpoint.
- Press ``l`` once: if the robot is not at the neutral pose (all-zero joints),
  command a homing move. Press ``l`` again once homed to capture a calibration
  offset from the current wrist pose.
"""

from __future__ import annotations

import argparse
import platform
import select
import socket
import sys
import time
from contextlib import contextmanager
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, Sequence

if platform.system() == "Windows":
    import msvcrt
else:
    import termios
    import tty

import numpy as np
import pinocchio as pin

@dataclass
class WristPose:
    translation: np.ndarray
    quaternion: np.ndarray  # [x, y, z, w]


@dataclass
class ParsedPacket:
    frame_idx: int | None
    wrists: Dict[str, WristPose]
    hand_joints: Dict[str, list[float]]


@dataclass
class CalibrationState:
    has_offset: bool = False
    offset: pin.SE3 | None = None


def parse_args(argv: Sequence[str]) -> argparse.Namespace:
    repo_root = Path(__file__).resolve().parents[1]
    default_urdf = repo_root / "urdf" / "M_lrmate_with_unijoint_hand.urdf"

    parser = argparse.ArgumentParser(description="Convert wrist poses to joint commands over UDP")
    parser.add_argument("--urdf", type=Path, default=default_urdf, help="Path to the robot URDF")
    parser.add_argument("--frame", default="wrist", help="End-effector frame name for Cartesian control")
    parser.add_argument("--listen-ip", default="0.0.0.0", help="IP to bind for incoming wrist packets")
    parser.add_argument("--listen-port", type=int, default=16000, help="UDP port for wrist/hand pose packets")
    parser.add_argument("--dest-ip", default="127.0.0.1", help="Destination IP for joint UDP stream")
    parser.add_argument("--dest-port", type=int, default=15000, help="Destination UDP port for arm joints")
    parser.add_argument("--dest-hand-port", type=int, default=15001, help="Destination UDP port for dexterous-hand joints")
    parser.add_argument("--sides", nargs="+", choices=["left", "right"], default=["left", "right"], help="Arms to drive")
    parser.add_argument("--max-iters", type=int, default=100, help="Max IK iterations per update")
    parser.add_argument("--tolerance", type=float, default=1e-4, help="IK convergence tolerance on SE(3) error")
    parser.add_argument("--damping", type=float, default=1e-3, help="Damping factor for pseudo-inverse")
    parser.add_argument("--home-tolerance", type=float, default=1e-2, help="Tolerance (deg) for considering joints at home")
    parser.add_argument("--rate-hz", type=float, default=60.0, help="Simulation loop rate in Hz")
    return parser.parse_args(argv)


def _normalize_quaternion(q: Sequence[float]) -> np.ndarray:
    quat = np.array(q, dtype=float)
    norm = np.linalg.norm(quat)
    if norm == 0:
        return np.array([0.0, 0.0, 0.0, 1.0])
    return quat / norm


def _parse_pose_packet(packet: bytes) -> ParsedPacket:
    decoded = packet.decode("utf-8", errors="ignore")
    lines = [line.strip() for line in decoded.splitlines() if line.strip()]

    frame_idx: int | None = None
    wrists: Dict[str, WristPose] = {}
    hand_joints: Dict[str, list[float]] = {}

    for line in lines:
        parts = [part.strip() for part in line.split(",") if part.strip()]
        if len(parts) < 2:
            continue

        prefix = parts[0].lower()
        if prefix == "frame":
            try:
                frame_idx = int(parts[1])
            except ValueError:
                frame_idx = None
            continue

        if len(parts) < 3:
            continue

        side = parts[1].lower()
        if prefix == "wrist" and len(parts) >= 9:
            try:
                values = [float(value) for value in parts[2:9]]
            except ValueError:
                continue
            translation = np.array(values[:3], dtype=float)
            quaternion = _normalize_quaternion([values[3], values[4], values[5], values[6]])
            wrists[side] = WristPose(translation=translation, quaternion=quaternion)
        elif prefix == "hand":
            try:
                hand_joints[side] = [float(value) for value in parts[2:]]
            except ValueError:
                continue

    return ParsedPacket(frame_idx=frame_idx, wrists=wrists, hand_joints=hand_joints)


def _solve_ik(
    model: pin.Model,
    frame_id: int,
    target: pin.SE3,
    seed: np.ndarray,
    *,
    max_iters: int,
    tolerance: float,
    damping: float,
) -> tuple[np.ndarray, bool]:
    q = seed.copy()
    data = model.createData()

    for _ in range(max_iters):
        pin.forwardKinematics(model, data, q)
        pin.updateFramePlacements(model, data)
        current = data.oMf[frame_id]

        error = pin.log6(current.actInv(target)).vector
        if np.linalg.norm(error) < tolerance:
            return q, True

        jacobian = pin.computeFrameJacobian(model, data, q, frame_id, pin.ReferenceFrame.LOCAL)
        jjt = jacobian @ jacobian.T
        dq = jacobian.T @ np.linalg.solve(jjt + damping * np.eye(6), error)
        q = pin.integrate(model, q, dq)

    return q, False


def _format_joint_packet(frame_idx: int, sides: Iterable[str], arm_deg: Dict[str, Sequence[float]]) -> str:
    lines = [f"frame,{frame_idx}"]
    for side in sides:
        joints = arm_deg.get(side, [])
        formatted = ",".join(f"{value:.6f}" for value in joints)
        lines.append(f"joint,{side},{formatted}")
    return "\n".join(lines)


@contextmanager
def raw_terminal_mode() -> None:
    """Context manager that enables non-blocking keyboard polling cross-platform."""

    if platform.system() == "Windows":
        # Windows console reads are already unbuffered when using ``msvcrt``.
        try:
            yield
        finally:
            return

    old_attrs = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    try:
        yield
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_attrs)


def _poll_keys() -> tuple[bool, bool]:
    hold_k = False
    trigger_l = False

    if platform.system() == "Windows":
        while msvcrt.kbhit():
            ch = msvcrt.getwch()
            if ch.lower() == "k":
                hold_k = True
            elif ch.lower() == "l":
                trigger_l = True
    else:
        rlist, _, _ = select.select([sys.stdin], [], [], 0)
        while rlist:
            ch = sys.stdin.read(1)
            if ch.lower() == "k":
                hold_k = True
            elif ch.lower() == "l":
                trigger_l = True
            rlist, _, _ = select.select([sys.stdin], [], [], 0)

    return hold_k, trigger_l


def _home_commands(sides: Iterable[str]) -> Dict[str, list[float]]:
    return {side: [0.0] * 6 for side in sides}


def _at_home(commands: Dict[str, Sequence[float]], *, tolerance_deg: float) -> bool:
    for angles in commands.values():
        for value in angles:
            if abs(value) > tolerance_deg:
                return False
    return True


def _hand_packet(frame_idx: int, sides: Iterable[str], joints: Dict[str, Sequence[float]]) -> str:
    lines = [f"frame,{frame_idx}"]
    for side in sides:
        values = joints.get(side, [])
        formatted = ",".join(f"{value:.6f}" for value in values)
        lines.append(f"hand,{side},{formatted}")
    return "\n".join(lines)


def _se3_from_wrist(pose: WristPose) -> pin.SE3:
    quat = pin.Quaternion(pose.quaternion[3], pose.quaternion[0], pose.quaternion[1], pose.quaternion[2])
    quat.normalize()
    rotation = quat.toRotationMatrix()
    return pin.SE3(rotation, pose.translation)


def stream_wrist_to_ik(args: argparse.Namespace) -> None:
    urdf_path = args.urdf.resolve()
    if not urdf_path.exists():
        raise FileNotFoundError(f"URDF not found: {urdf_path}")

    model = pin.buildModelFromUrdf(str(urdf_path))
    frame_id = model.getFrameId(args.frame)
    if frame_id == len(model.frames):
        available = ", ".join(frame.name for frame in model.frames)
        raise ValueError(f"Frame '{args.frame}' not found in model. Available frames: {available}")

    neutral_q = pin.neutral(model)
    data = model.createData()
    pin.forwardKinematics(model, data, neutral_q)
    pin.updateFramePlacements(model, data)
    robot_home_pose = data.oMf[frame_id]

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((args.listen_ip, args.listen_port))
    sock.setblocking(False)

    arm_sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    hand_sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    dest = (args.dest_ip, args.dest_port)
    hand_dest = (args.dest_ip, args.dest_hand_port)

    print(
        f"Listening for wrist poses on {args.listen_ip}:{args.listen_port}; "
        f"re-streaming IK joints to {args.dest_ip}:{args.dest_port} (hand port {args.dest_hand_port})."
    )

    calibrations: Dict[str, CalibrationState] = {side: CalibrationState() for side in args.sides}
    seeds: Dict[str, np.ndarray] = {side: neutral_q.copy() for side in args.sides}
    last_commands: Dict[str, list[float]] = {side: [0.0] * 6 for side in args.sides}
    homing_mode = False
    frame_idx = 0
    period_s = 1.0 / args.rate_hz if args.rate_hz > 0 else 0.0

    latest_packet = ParsedPacket(frame_idx=None, wrists={}, hand_joints={})

    with raw_terminal_mode():
        try:
            while True:
                try:
                    packet, _ = sock.recvfrom(4096)
                    latest_packet = _parse_pose_packet(packet)
                except BlockingIOError:
                    pass

                hold_k, trigger_l = _poll_keys()
                robot_at_home = _at_home(last_commands, tolerance_deg=args.home_tolerance)

                if trigger_l:
                    if not robot_at_home:
                        homing_mode = True
                        print("[wrist-ik] Homing robot to zero joints before calibration...")
                    else:
                        for side, pose in latest_packet.wrists.items():
                            if side not in args.sides:
                                continue
                            device_home = _se3_from_wrist(pose)
                            offset = robot_home_pose * device_home.inverse()
                            calibrations[side] = CalibrationState(has_offset=True, offset=offset)
                            print(f"[wrist-ik] Captured calibration offset for {side} wrist.")

                arm_commands: Dict[str, list[float]] = {}
                hand_commands: Dict[str, list[float]] = {}

                if homing_mode:
                    arm_commands = _home_commands(args.sides)
                elif hold_k and latest_packet.wrists:
                    for side, pose in latest_packet.wrists.items():
                        if side not in args.sides:
                            continue

                        seed = seeds[side]
                        target = _se3_from_wrist(pose)
                        calibration = calibrations.get(side)
                        if calibration and calibration.has_offset and calibration.offset is not None:
                            target = calibration.offset * target

                        q_sol, converged = _solve_ik(
                            model,
                            frame_id,
                            target,
                            seed,
                            max_iters=args.max_iters,
                            tolerance=args.tolerance,
                            damping=args.damping,
                        )
                        if converged:
                            seeds[side] = q_sol
                            arm_commands[side] = [float(np.degrees(value)) for value in q_sol[:6]]
                        else:
                            arm_commands[side] = last_commands.get(side, [0.0] * 6)
                            print(f"[wrist-ik][WARN] IK failed for {side} at frame {frame_idx}; sending last valid command.")

                    for side, joints in latest_packet.hand_joints.items():
                        if side in args.sides:
                            hand_commands[side] = joints

                if arm_commands:
                    payload = _format_joint_packet(frame_idx, arm_commands.keys(), arm_commands)
                    arm_sender.sendto(payload.encode("utf-8"), dest)
                    last_commands.update(arm_commands)

                if hand_commands:
                    hand_payload = _hand_packet(frame_idx, hand_commands.keys(), hand_commands)
                    hand_sender.sendto(hand_payload.encode("utf-8"), hand_dest)

                if homing_mode and _at_home(last_commands, tolerance_deg=args.home_tolerance):
                    homing_mode = False
                    print("[wrist-ik] Homing complete; press 'l' again to capture calibration.")

                frame_idx += 1

                if period_s > 0:
                    time.sleep(period_s)
        except KeyboardInterrupt:
            print("\n[wrist-ik] Exiting wrist teleop IK streamer.")


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv or sys.argv[1:])
    stream_wrist_to_ik(args)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
