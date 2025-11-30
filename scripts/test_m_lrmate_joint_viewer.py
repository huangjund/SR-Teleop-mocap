"""Compact viewer for the M_lrmate URDF with live joint coordinates."""
from __future__ import annotations

import time
from pathlib import Path

import pybullet as p
import pybullet_data


def main() -> None:
    repo_root = Path(__file__).resolve().parents[1]
    urdf_path = repo_root / "urdf" / "M_lrmate_with_unijoint_hand.urdf"

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
