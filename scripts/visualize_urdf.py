"""Simple URDF visualizer using PyBullet.

Usage examples:
    python scripts/visualize_urdf.py --urdf urdf/lrmate_with_unijoint_hand.urdf
    python scripts/visualize_urdf.py --urdf urdf/gripper_2/gripper.urdf --free-base
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path
from typing import Iterable, Sequence

import pybullet as p
import pybullet_data


def _normalize_triple(values: Sequence[float], *, default: float = 0.0) -> tuple[float, float, float]:
    padded = list(values[:3]) + [default] * (3 - len(values))
    return tuple(padded[:3])  # type: ignore[return-value]


def _set_search_paths(paths: Iterable[Path]) -> None:
    """Register mesh search paths for relative URDF resources.

    PyBullet keeps only one active additional search path; the last call wins.
    To maximize compatibility, we set the most likely mesh directories last so
    their entries take precedence.
    """

    ordered_paths = [Path(pybullet_data.getDataPath())]
    for path in paths:
        ordered_paths.append(path)
        ordered_paths.append(path / "meshes")
        ordered_paths.append(path / "CAD")

    for path in ordered_paths:
        if path.exists():
            p.setAdditionalSearchPath(str(path))


def parse_args(argv: Sequence[str]) -> argparse.Namespace:
    repo_root = Path(__file__).resolve().parents[1]
    default_urdf = repo_root / "urdf" / "lrmate_with_unijoint_hand.urdf"

    parser = argparse.ArgumentParser(description="Visualize a URDF using PyBullet")
    parser.add_argument("--urdf", type=Path, default=default_urdf, help="Path to the URDF file to load")
    parser.add_argument("--base-xyz", type=float, nargs="*", default=(0.0, 0.0, 0.0), help="Initial base position (x y z)")
    parser.add_argument("--base-rpy", type=float, nargs="*", default=(0.0, 0.0, 0.0), help="Initial base roll/pitch/yaw in radians")
    parser.add_argument("--free-base", action="store_true", help="Allow the base to move (default: fixed base)")
    parser.add_argument("--no-plane", action="store_true", help="Do not add a ground plane")
    parser.add_argument("--realtime", action="store_true", help="Run the simulation in realtime instead of stepping manually")
    parser.add_argument("--camera-distance", type=float, default=2.2, help="Camera distance from the target")
    parser.add_argument("--camera-yaw", type=float, default=135.0, help="Camera yaw angle in degrees")
    parser.add_argument("--camera-pitch", type=float, default=-25.0, help="Camera pitch angle in degrees")
    parser.add_argument("--camera-target", type=float, nargs="*", default=(0.0, 0.0, 0.7), help="Camera target position (x y z)")
    return parser.parse_args(argv)


def visualize(args: argparse.Namespace) -> None:
    urdf_path = args.urdf.resolve()
    if not urdf_path.exists():
        raise FileNotFoundError(f"URDF not found: {urdf_path}")

    base_position = _normalize_triple(args.base_xyz)
    base_orientation = p.getQuaternionFromEuler(_normalize_triple(args.base_rpy))
    camera_target = _normalize_triple(args.camera_target)

    client = p.connect(p.GUI)
    try:
        p.resetSimulation(physicsClientId=client)
        _set_search_paths([urdf_path.parent])
        p.setGravity(0, 0, -9.81, physicsClientId=client)

        if not args.no_plane:
            p.loadURDF("plane.urdf", useFixedBase=True, physicsClientId=client)

        p.resetDebugVisualizerCamera(
            cameraDistance=args.camera_distance,
            cameraYaw=args.camera_yaw,
            cameraPitch=args.camera_pitch,
            cameraTargetPosition=camera_target,
            physicsClientId=client,
        )

        robot_id = p.loadURDF(
            str(urdf_path),
            basePosition=base_position,
            baseOrientation=base_orientation,
            useFixedBase=not args.free_base,
            flags=p.URDF_USE_INERTIA_FROM_FILE,
            physicsClientId=client,
        )

        joint_count = p.getNumJoints(robot_id, physicsClientId=client)
        joint_names = [p.getJointInfo(robot_id, i, physicsClientId=client)[1].decode("utf-8") for i in range(joint_count)]
        print(f"Loaded {urdf_path.name} with {joint_count} joints:")
        for index, name in enumerate(joint_names):
            print(f"  [{index}] {name}")

        if args.realtime:
            p.setRealTimeSimulation(1, physicsClientId=client)
            print("Running in realtime mode. Close the window or press Ctrl+C to exit.")
            while True:
                time.sleep(0.1)
        else:
            print("Stepping simulation at 240 Hz. Close the window or press Ctrl+C to exit.")
            while True:
                p.stepSimulation(physicsClientId=client)
                time.sleep(1.0 / 240.0)
    except KeyboardInterrupt:
        print("Exiting visualization.")
    finally:
        p.disconnect(physicsClientId=client)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv or sys.argv[1:])
    visualize(args)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
