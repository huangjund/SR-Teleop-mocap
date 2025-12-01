"""Compact viewer for the M_lrmate URDF with live joint coordinates."""
from __future__ import annotations

import time
from pathlib import Path
from typing import Dict, List

import pybullet as p
import pybullet_data


def _draw_frame(
    name: str,
    position: List[float],
    orientation: List[float],
    client: int,
    cache: Dict[str, List[int]],
    length: float = 0.15,
) -> None:
    """Draw a triad at ``position`` with axes aligned to ``orientation``."""

    rot = p.getMatrixFromQuaternion(orientation)
    axes = [
        (rot[0], rot[3], rot[6]),  # x
        (rot[1], rot[4], rot[7]),  # y
        (rot[2], rot[5], rot[8]),  # z
    ]
    colors = [(1, 0, 0), (0, 1, 0), (0, 0, 1)]
    ids = cache.get(name, [None, None, None])

    for i, axis in enumerate(axes):
        end = [position[j] + length * axis[j] for j in range(3)]
        ids[i] = p.addUserDebugLine(
            position,
            end,
            colors[i],
            lineWidth=2.0,
            lifeTime=0,
            replaceItemUniqueId=ids[i] if ids[i] is not None else -1,
            physicsClientId=client,
        )

    cache[name] = ids


def main() -> None:
    repo_root = Path(__file__).resolve().parents[1]
    urdf_path = repo_root / "urdf" / "M_lrmate_with_unijoint_hand.urdf"#"lrmate_without_hand.urdf"

    client = p.connect(p.GUI)
    try:
        p.resetSimulation(physicsClientId=client)
        p.setAdditionalSearchPath(str(urdf_path.parent), physicsClientId=client)
        p.setAdditionalSearchPath(str(pybullet_data.getDataPath()), physicsClientId=client)
        p.setGravity(0, 0, -9.81, physicsClientId=client)

        p.loadURDF("plane.urdf", useFixedBase=True, physicsClientId=client)
        robot_id = p.loadURDF(
            str(urdf_path),
            basePosition=(0, 0, 0),
            useFixedBase=True,
            flags=p.URDF_USE_INERTIA_FROM_FILE,
            physicsClientId=client,
        )

        movable = []
        sliders = []
        for i in range(p.getNumJoints(robot_id, physicsClientId=client)):
            info = p.getJointInfo(robot_id, i, physicsClientId=client)
            if info[2] in (p.JOINT_REVOLUTE, p.JOINT_PRISMATIC):
                lower, upper = info[8:10]
                lower = lower if lower < upper else -3.14
                upper = upper if lower < upper else 3.14
                sliders.append(p.addUserDebugParameter(info[1].decode(), lower, upper, 0.0, physicsClientId=client))
                movable.append((i, info[1].decode()))

        text_id = None
        frame_cache: Dict[str, List[int]] = {}

        print(f"Loaded {urdf_path.name} with {len(movable)} controllable joints.")
        while True:
            coords = []
            for (joint_index, name), slider in zip(movable, sliders):
                target = p.readUserDebugParameter(slider, physicsClientId=client)
                p.setJointMotorControl2(
                    robot_id,
                    joint_index,
                    p.POSITION_CONTROL,
                    targetPosition=target,
                    force=500,
                    physicsClientId=client,
                )
                pos = p.getJointState(robot_id, joint_index, physicsClientId=client)[0]
                coords.append(f"{name}:{pos:.3f}")

            base_pos, base_orn = p.getBasePositionAndOrientation(robot_id, physicsClientId=client)
            _draw_frame("base", base_pos, base_orn, client, frame_cache)

            for link_index in range(p.getNumJoints(robot_id, physicsClientId=client)):
                link_state = p.getLinkState(
                    robot_id,
                    link_index,
                    computeForwardKinematics=True,
                    physicsClientId=client,
                )
                if link_state is not None:
                    _draw_frame(
                        f"joint_{link_index}",
                        list(link_state[4]),
                        list(link_state[5]),
                        client,
                        frame_cache,
                    )

            if text_id is not None:
                p.removeUserDebugItem(text_id, physicsClientId=client)
            text_id = p.addUserDebugText(
                " | ".join(coords),
                [0, 0, 1.2],
                textSize=1.5,
                lifeTime=0,
                physicsClientId=client,
            )
            p.stepSimulation(physicsClientId=client)
            time.sleep(1.0 / 240.0)
    except KeyboardInterrupt:
        pass
    finally:
        p.disconnect(physicsClientId=client)


if __name__ == "__main__":
    main()
