"""Interactive Cartesian slider that solves IK and streams joints and wrist poses.

This tool complements ``scripts/fake_teleop_slider.py`` by letting you control
an end-effector pose directly. The sliders define a 6D Cartesian target
(x, y, z, roll, pitch, yaw) for the Fanuc wrist; each target is converted to
joint angles with Pinocchio inverse kinematics and broadcast to the same UDP
endpoint as the joint-space fake teleop while also sending wrist poses to
``udp_wrist_to_ik.py``.
"""

from __future__ import annotations

import argparse
import socket
import sys
import tkinter as tk
from pathlib import Path
from typing import Dict, Iterable, Sequence

import numpy as np
import pinocchio as pin

PoseVector = Sequence[float]


def parse_args(argv: Sequence[str]) -> argparse.Namespace:
    repo_root = Path(__file__).resolve().parents[1]
    default_urdf = repo_root / "urdf" / "M_lrmate_with_unijoint_hand.urdf"

    parser = argparse.ArgumentParser(description="Send fake Cartesian teleop packets with IK")
    parser.add_argument("--urdf", type=Path, default=default_urdf, help="Path to the robot URDF")
    parser.add_argument("--frame", default="wrist", help="End-effector frame name for Cartesian control")
    parser.add_argument("--dest-ip", default="127.0.0.1", help="Destination IP for the UDP stream")
    parser.add_argument("--dest-port", type=int, default=15002, help="Destination UDP port for arm joints")
    parser.add_argument(
        "--wrist-port", type=int, default=16000, help="Destination UDP port for wrist pose packets"
    )
    parser.add_argument("--rate-hz", type=float, default=30.0, help="Streaming rate in Hz")
    parser.add_argument(
        "--sides", nargs="+", choices=["left", "right"], default=["left", "right"], help="Arms to stream"
    )
    parser.add_argument("--position-limit", type=float, default=1.0, help="Symmetric slider limit in meters")
    parser.add_argument("--rotation-limit", type=float, default=3.14, help="Symmetric slider limit in radians")
    parser.add_argument("--max-iters", type=int, default=100, help="Max IK iterations per update")
    parser.add_argument("--tolerance", type=float, default=1e-4, help="IK convergence tolerance on SE(3) error")
    parser.add_argument("--damping", type=float, default=1e-3, help="Damping factor for pseudo-inverse")
    return parser.parse_args(argv)


def _pose_from_sliders(sliders: Sequence[tk.Scale]) -> pin.SE3:
    xyz = np.array([float(sliders[i].get()) for i in range(3)])
    rpy = np.array([float(sliders[i].get()) for i in range(3, 6)])
    rotation = pin.rpy.rpyToMatrix(rpy)
    return pin.SE3(rotation, xyz)


def _default_pose_params(model: pin.Model, frame_id: int) -> PoseVector:
    q0 = pin.neutral(model)
    data = model.createData()
    pin.forwardKinematics(model, data, q0)
    pin.updateFramePlacements(model, data)
    pose = data.oMf[frame_id]
    rpy = pin.rpy.matrixToRpy(pose.rotation)
    return (*pose.translation.tolist(), *rpy.tolist())


def _create_sliders(
    root: tk.Tk, sides: Iterable[str], *, position_limit: float, rotation_limit: float, defaults: PoseVector
) -> Dict[str, list[tk.Scale]]:
    sliders: Dict[str, list[tk.Scale]] = {}
    labels = ["X (m)", "Y (m)", "Z (m)", "Roll (rad)", "Pitch (rad)", "Yaw (rad)"]
    limits = [position_limit, position_limit, position_limit, rotation_limit, rotation_limit, rotation_limit]

    for side in sides:
        frame = tk.LabelFrame(root, text=f"{side.capitalize()} wrist")
        frame.pack(fill=tk.BOTH, expand=True, padx=8, pady=4)

        sliders[side] = []
        for idx, (label_text, limit, default_value) in enumerate(zip(labels, limits, defaults)):
            label = tk.Label(frame, text=label_text)
            label.grid(row=idx, column=0, sticky=tk.W, padx=6, pady=2)

            scale = tk.Scale(
                frame,
                from_=-limit,
                to=limit,
                resolution=0.001 if idx < 3 else 0.01,
                orient=tk.HORIZONTAL,
                length=320,
            )
            scale.set(default_value)
            scale.grid(row=idx, column=1, sticky=tk.EW, padx=6, pady=2)
            sliders[side].append(scale)

        frame.columnconfigure(1, weight=1)

    return sliders


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


def _gather_targets(sliders: Dict[str, list[tk.Scale]]) -> Dict[str, pin.SE3]:
    return {side: _pose_from_sliders(scales) for side, scales in sliders.items()}


def _format_packet(frame_idx: int, sides: Iterable[str], angles_deg: Dict[str, Sequence[float]]) -> str:
    lines = [f"frame,{frame_idx}"]
    for side in sides:
        joint_values = angles_deg.get(side, [])
        formatted = ",".join(f"{value:.6f}" for value in joint_values)
        lines.append(f"joint,{side},{formatted}")
    return "\n".join(lines)


def _to_degrees(angles_rad: Dict[str, Sequence[float]]) -> Dict[str, list[float]]:
    return {side: [float(np.degrees(value)) for value in values] for side, values in angles_rad.items()}


def _format_wrist_packet(frame_idx: int, sides: Iterable[str], poses: Dict[str, pin.SE3]) -> str:
    """Create a UDP payload carrying wrist translations and quaternions."""

    lines = [f"frame,{frame_idx}"]
    for side in sides:
        pose = poses.get(side)
        if pose is None:
            continue

        translation = pose.translation
        quat = pin.Quaternion(pose.rotation)
        quat.normalize()
        quaternion = (quat.x, quat.y, quat.z, quat.w)
        values = [*translation, *quaternion]
        formatted = ",".join(f"{value:.6f}" for value in values)
        lines.append(f"wrist,{side},{formatted}")

    return "\n".join(lines)


def stream_fake_cartesian(args: argparse.Namespace) -> None:
    model = pin.buildModelFromUrdf(str(args.urdf))
    frame_id = model.getFrameId(args.frame)
    if frame_id == len(model.frames):
        available = ", ".join(frame.name for frame in model.frames)
        raise ValueError(f"Frame '{args.frame}' not found in model. Available frames: {available}")

    default_pose = _default_pose_params(model, frame_id)

    root = tk.Tk()
    root.title("Fake Cartesian teleop slider")

    sliders = _create_sliders(
        root,
        args.sides,
        position_limit=args.position_limit,
        rotation_limit=args.rotation_limit,
        defaults=default_pose,
    )

    seeds = {side: pin.neutral(model) for side in args.sides}
    last_commands_rad: Dict[str, list[float]] = {side: seeds[side][:6].tolist() for side in args.sides}

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    joint_destination = (args.dest_ip, args.dest_port)
    wrist_destination = (args.dest_ip, args.wrist_port)

    print(
        "Streaming fake Cartesian teleop packets to "
        f"{args.dest_ip}:{args.dest_port} (joints) and {args.dest_ip}:{args.wrist_port} (wrist) "
        f"for sides: {', '.join(args.sides)}. Close the window or press Ctrl+C to stop."
    )

    frame_idx = 0
    running = True
    period_ms = int(1000.0 / args.rate_hz) if args.rate_hz > 0 else 0

    def on_close() -> None:
        nonlocal running
        running = False
        root.destroy()

    def send_packet() -> None:
        nonlocal frame_idx
        if not running:
            return

        targets = _gather_targets(sliders)
        for side, target in targets.items():
            seed = seeds[side]
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
                last_commands_rad[side] = q_sol[:6].tolist()
            else:
                print(f"[WARN] IK failed to converge for {side} at frame {frame_idx}; sending last valid joints.")

        joint_payload = _format_packet(frame_idx, args.sides, _to_degrees(last_commands_rad))
        wrist_payload = _format_wrist_packet(frame_idx, args.sides, targets)
        print(f"[Frame {frame_idx}] Sending wrist packet:\n{wrist_payload}")
        sock.sendto(joint_payload.encode("utf-8"), joint_destination)
        sock.sendto(wrist_payload.encode("utf-8"), wrist_destination)

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
        print("Stopping fake Cartesian teleop sender.")
        on_close()


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv or sys.argv[1:])
    stream_fake_cartesian(args)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
