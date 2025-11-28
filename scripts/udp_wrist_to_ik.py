"""Listen for robot joint angles over UDP and drive a Fanuc arm in PyBullet.

C++ side: mocap_teleop ingests the Axis Studio BVH stream, extracts each wrist
pose, feeds it through the FastIK package, retargets the dexterous hand, and
broadcasts joint vectors over UDP as newline-delimited CSV:

    frame,<frame_index>
    joint,<side>,<arm_joint0>,...,<arm_joint5>
    hand,<side>,<hand_joint0>,...

Python side (this script): bind UDP sockets, parse those joint vectors, and
apply them directly to a URDF arm for visual validation of the IK output. Both
the arm and the dexterous hand are driven when streams are available.
"""

from __future__ import annotations

import argparse
import socket
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, Sequence

import pybullet as p
import pybullet_data


def _normalize_triple(values: Sequence[float], *, default: float = 0.0) -> tuple[float, float, float]:
    padded = list(values[:3]) + [default] * (3 - len(values))
    return tuple(padded[:3])  # type: ignore[return-value]


def _set_search_paths(paths: Iterable[Path]) -> None:
    """Register mesh search paths for relative URDF resources."""

    ordered_paths: list[Path] = []
    for path in paths:
        ordered_paths.append(path)
        ordered_paths.append(path / "meshes")
        ordered_paths.append(path / "CAD")
    ordered_paths.append(Path(pybullet_data.getDataPath()))

    for path in ordered_paths:
        if path.exists():
            p.setAdditionalSearchPath(str(path))


def _parse_joint_packet(packet: bytes) -> tuple[Dict[str, list[float]], Dict[str, list[float]]]:
    """Decode arm and dexterous-hand datagrams from the C++ teleop sender."""

    decoded = packet.decode("utf-8", errors="ignore")
    lines = [line.strip() for line in decoded.splitlines() if line.strip()]

    arm_targets: Dict[str, list[float]] = {}
    hand_targets: Dict[str, list[float]] = {}
    for line in lines[1:]:
        parts = line.split(",")
        if len(parts) < 3:
            continue

        prefix = parts[0].strip().lower()
        side = parts[1].strip().lower()
        try:
            angles = [float(value) for value in parts[2:]]
        except ValueError:
            continue

        if prefix == "joint":
            arm_targets[side] = angles
        elif prefix == "hand":
            hand_targets[side] = angles

    return arm_targets, hand_targets


def parse_args(argv: Sequence[str]) -> argparse.Namespace:
    repo_root = Path(__file__).resolve().parents[1]
    default_urdf = repo_root / "urdf" / "lrmate_with_unijoint_hand.urdf"

    parser = argparse.ArgumentParser(description="Visualize Fanuc joint commands via UDP")
    parser.add_argument("--urdf", type=Path, default=default_urdf, help="Path to the URDF file to load")
    parser.add_argument("--listen-ip", default="0.0.0.0", help="IP to bind the UDP listener")
    parser.add_argument("--listen-port", type=int, default=15000, help="UDP port that the C++ teleop program streams to")
    parser.add_argument("--listen-hand-port", type=int, default=15001, help="UDP port carrying dexterous-hand joint angles")
    parser.add_argument("--no-hand", action="store_true", help="Ignore dexterous-hand UDP packets")
    parser.add_argument(
        "--sides",
        nargs="+",
        choices=["left", "right"],
        default=["left", "right"],
        help="Which arms to spawn and drive (default: both)",
    )
    parser.add_argument("--left-base-xyz", type=float, nargs="*", default=(-0.6, 0.3, 0.0), help="Left base position")
    parser.add_argument("--left-base-rpy", type=float, nargs="*", default=(0.0, 0.0, 0.0), help="Left base roll/pitch/yaw")
    parser.add_argument("--right-base-xyz", type=float, nargs="*", default=(0.6, 0.3, 0.0), help="Right base position")
    parser.add_argument("--right-base-rpy", type=float, nargs="*", default=(0.0, 0.0, 0.0), help="Right base roll/pitch/yaw")
    parser.add_argument("--no-plane", action="store_true", help="Do not add a ground plane")
    return parser.parse_args(argv)


def _controlled_joint_indices(robot_id: int) -> list[int]:
    indices: list[int] = []
    for joint_id in range(p.getNumJoints(robot_id)):
        joint_type = p.getJointInfo(robot_id, joint_id)[2]
        if joint_type != p.JOINT_FIXED:
            indices.append(joint_id)
    return indices


@dataclass
class ArmInstance:
    side: str
    robot_id: int
    arm_joint_indices: list[int]
    hand_joint_indices: list[int]
    arm_targets: list[float]
    hand_targets: list[float]


def _spawn_arm(side: str, urdf_path: Path, base_xyz: Sequence[float], base_rpy: Sequence[float], client: int) -> ArmInstance:
    base_position = _normalize_triple(base_xyz)
    base_orientation = p.getQuaternionFromEuler(_normalize_triple(base_rpy))

    robot_id = p.loadURDF(
        str(urdf_path),
        basePosition=base_position,
        baseOrientation=base_orientation,
        useFixedBase=True,
        flags=p.URDF_USE_INERTIA_FROM_FILE,
        physicsClientId=client,
    )

    controlled_joints = _controlled_joint_indices(robot_id)
    arm_joint_indices = controlled_joints[:6]
    hand_joint_indices = controlled_joints[6:]

    arm_targets = [p.getJointState(robot_id, j)[0] for j in arm_joint_indices]
    hand_targets = [p.getJointState(robot_id, j)[0] for j in hand_joint_indices]
    return ArmInstance(
        side=side,
        robot_id=robot_id,
        arm_joint_indices=arm_joint_indices,
        hand_joint_indices=hand_joint_indices,
        arm_targets=arm_targets,
        hand_targets=hand_targets,
    )


def visualize_stream(args: argparse.Namespace) -> None:
    urdf_path = args.urdf.resolve()
    if not urdf_path.exists():
        raise FileNotFoundError(f"URDF not found: {urdf_path}")

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((args.listen_ip, args.listen_port))
    sock.setblocking(False)
    print(f"Listening for arm joints on {args.listen_ip}:{args.listen_port} (sides={','.join(args.sides)})")

    hand_sock = None
    if not args.no_hand:
        hand_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        hand_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        hand_sock.bind((args.listen_ip, args.listen_hand_port))
        hand_sock.setblocking(False)
        print(f"Listening for dexterous-hand joints on {args.listen_ip}:{args.listen_hand_port}")

    client = p.connect(p.GUI)
    try:
        p.resetSimulation(physicsClientId=client)
        _set_search_paths([urdf_path.parent])
        p.setGravity(0, 0, -9.81, physicsClientId=client)

        if not args.no_plane:
            plane_path = Path(pybullet_data.getDataPath()) / "plane.urdf"
            p.loadURDF(str(plane_path), useFixedBase=True, physicsClientId=client)

        arms: Dict[str, ArmInstance] = {}
        for side in args.sides:
            if side == "left":
                arms[side] = _spawn_arm(side, urdf_path, args.left_base_xyz, args.left_base_rpy, client)
            else:
                arms[side] = _spawn_arm(side, urdf_path, args.right_base_xyz, args.right_base_rpy, client)

        if not arms:
            raise RuntimeError("No arms selected; specify --sides left and/or right")

        print(
            f"Loaded {len(arms)} arm(s) from {urdf_path.name}. "
            "Waiting for joint datagrams... Close the window or press Ctrl+C to exit."
        )

        while True:
            try:
                packet, _ = sock.recvfrom(4096)
            except BlockingIOError:
                packet = b""

            if packet:
                arm_cmds, _ = _parse_joint_packet(packet)
                for side, instance in arms.items():
                    if side not in arm_cmds:
                        continue

                    angles = arm_cmds[side]
                    if len(angles) < len(instance.arm_joint_indices):
                        angles = angles + [0.0] * (len(instance.arm_joint_indices) - len(angles))
                    instance.arm_targets = angles[: len(instance.arm_joint_indices)]

            if hand_sock is not None:
                try:
                    hand_packet, _ = hand_sock.recvfrom(4096)
                except BlockingIOError:
                    hand_packet = b""

                if hand_packet:
                    _, hand_cmds = _parse_joint_packet(hand_packet)
                    for side, instance in arms.items():
                        if side not in hand_cmds:
                            continue

                        angles = hand_cmds[side]
                        if len(angles) < len(instance.hand_joint_indices):
                            angles = angles + [0.0] * (len(instance.hand_joint_indices) - len(angles))
                        instance.hand_targets = angles[: len(instance.hand_joint_indices)]

            for arm in arms.values():
                for idx, joint_id in enumerate(arm.arm_joint_indices):
                    target = arm.arm_targets[idx] if idx < len(arm.arm_targets) else 0.0
                    p.setJointMotorControl2(
                        arm.robot_id,
                        joint_id,
                        p.POSITION_CONTROL,
                        targetPosition=target,
                        force=500,
                        physicsClientId=client,
                    )

                for idx, joint_id in enumerate(arm.hand_joint_indices):
                    target = arm.hand_targets[idx] if idx < len(arm.hand_targets) else 0.0
                    p.setJointMotorControl2(
                        arm.robot_id,
                        joint_id,
                        p.POSITION_CONTROL,
                        targetPosition=target,
                        force=100,
                        physicsClientId=client,
                    )

            p.stepSimulation(physicsClientId=client)
            time.sleep(1.0 / 240.0)
    except KeyboardInterrupt:
        print("Exiting UDP joint visualizer.")
    finally:
        sock.close()
        if hand_sock is not None:
            hand_sock.close()
        p.disconnect(physicsClientId=client)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv or sys.argv[1:])
    visualize_stream(args)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
