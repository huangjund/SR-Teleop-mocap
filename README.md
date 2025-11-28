# MocapApi
MocapApi for Axis Studio of Noitom

## Document
English Documentation : https://mocap-api.noitom.com/mocap_api_en.html

## MocapAPI data you can retrieve
- **Rigid bodies** – orientation (quaternion), position, status, ID, joint tag, and axis-angle rotation through `IMCPRigidBody` handles.
- **Trackers** – send messages and query device rotations, positions, Euler angles, device counts, and names via `IMCPTracker`.
- **Sensor modules** – per-module posture quaternion, angular velocity, acceleration, ID, compass value, temperature, and timestamps with `IMCPSensorModule`.
- **Body parts & joints** – joint/world positions, displacement speed, body-part posture quaternions, joint names/tags, hierarchy (parent/children), body-part and sensor-module links, local rotations (quaternion or Euler), default poses, grounding state, and groundable points via `IMCPBodyPart` and `IMCPJoint`.
- **Avatars** – avatar index, name, root joint, full joint lists, rigid-body handles, joint hierarchy strings, posture indices, and posture timestamps through `IMCPAvatar`.
- **Markers, PWR devices, and Alice bus** – marker positions (`IMCPMarker`), PWR ID/status/pose (`IMCPPWR`), and lists/timestamps for sensors, markers, rigid bodies, and PWR devices from `IMCPAliceHub`.
- **Event stream** – motion, rigid-body, sensor, tracker, marker, PWR, notify, and error events (with timestamps) enumerated in `MCPEvent_t`.
- **Settings and render settings** – networking (UDP/TCP) plus BVH rotation/data/transformation flags (`IMCPSettings`), and coordinate-system/up-vector/front-vector/unit/rotation-direction presets for output (`IMCPRenderSettings`).
- **Application control** – create/open/close applications, set settings/render settings, poll cached events, enumerate avatars/rigid bodies/sensors/trackers, queue commands, and register event handlers via `IMCPApplication`.

## Run Demo

- Run install.bat
- Use U3D or Unreal open demo

## BVH stream viewer

The default executable (`mocap_demo`) subscribes to Axis Studio BVH output, fits each frame to the
MocapApi skeleton, and visualizes the hierarchy. The viewer never renders the Axis BVH hierarchy
directly; every frame is re-targeted through `IMCPAvatar`/`IMCPJoint` so the drawn skeleton always
reflects what MocapApi reports. The client requests YXZ BVH rotation order and configures render
settings for the Axis “OPT” coordinate system (Y-up, +Z forward, right-handed, counter-clockwise
rotations).

```
cmake -S . -B build
cmake --build build --target mocap_demo
```

Then launch and point it at the Axis Studio BVH UDP endpoint (defaults shown):

```
./build/mocap_demo --server 127.0.0.1 --port 7012 --filter full
```

Use `--filter hands` or `--filter body` to toggle which parts of the skeleton are drawn.

## Bimanual teleoperation data stream

Build the wrist/hand extractor:

```
cmake -S . -B build
cmake --build build --target mocap_teleop
```

Run it against the Axis Studio BVH endpoint (same defaults as the viewer):

```
./build/mocap_teleop --server 127.0.0.1 --port 7012
```

### Step 1: C++ IK to Fanuc joint angles, Python visualization

For the arm-only milestone (with optional dexterous hand streaming), the
pipeline now mirrors the production flow:

1. The C++ teleop client ingests the BVH stream, extracts each wrist pose, and
   runs FastIK to compute Fanuc arm joint angles (no hand/gripper).
2. The resulting arm joint vectors are broadcast over UDP as newline-delimited
   CSV to a configurable destination. You can choose to stream `left`,
   `right`, or both arms.
3. Ergonomic finger angles are retargeted to a five-finger dexterous hand and
   streamed over a second UDP channel (default port `15001`).
4. The Python helper listens for those joint vectors and drives two URDF arms
   (and their attached hand joints) in PyBullet directly—no IK on the Python
   side—so you can verify the FastIK output and hand retargeting in realtime.

Each UDP packet looks like:

```
frame,<frame_index>
joint,<side>,<arm_joint0>,...,<arm_joint5>
hand,<side>,<hand_joint0>,...
```

Usage:

```bash
# Terminal 1: stream joint angles from BVH + FastIK (default streams both arms)
./build/mocap_teleop --server 127.0.0.1 --port 7012 --out-ip 127.0.0.1 --out-port 15000 --hand-port 15001

# Terminal 2: visualize both arms in PyBullet
python scripts/udp_wrist_to_ik.py --urdf urdf/lrmate_with_unijoint_hand.urdf --listen-port 15000 --listen-hand-port 15001
```

Override `--out-ip/--out-port` on the C++ side to target a different listener,
or add `--no-udp` if you only need console logging. Use `--no-hand-udp` on the
C++ side or `--no-hand` on the Python side to disable dexterous-hand streaming.

To stream/visualize just one arm, add a comma-separated `--sides` filter to the
C++ teleop program (e.g., `--sides left` or `--sides right`) and pass the same
flag to the Python helper. You can also offset the default arm bases with
`--left-base-xyz/--left-base-rpy` and `--right-base-xyz/--right-base-rpy` when
running the visualizer.

### What the stream provides
- **Wrist 6D poses** – both wrists are reported in world space (OPT basis, Y-up, +Z forward) as position plus orientation
  quaternions coming directly from MocapApi’s computed joint transforms.
- **Ergonomic hand joint angles** – every finger joint’s local quaternion is decomposed using the YXZ order; the angles are
  reported as flexion (X axis), abduction/splay (Y axis), and twist (Z axis) in degrees so they map cleanly to human-centric
  grasp DOFs.
- **Raw finger joint 6D poses** – call `TeleopMapping::RawFingerPoses()` to pull each finger joint’s world position and
  orientation if you need unprocessed values for custom retargeting or filtering.

### How ergonomic angles are derived
Each MocapApi joint delivers a local quaternion relative to its parent. The teleop program converts that quaternion into a YXZ
Euler triple (matching the requested BVH rotation order):

- **Flexion** = rotation about local X (curl/extension).
- **Abduction** = rotation about local Y (finger splay toward/away from neighbors).
- **Twist** = rotation about local Z (axial roll of the phalanx).

Those three angles are printed per joint every 30 frames; the wrists log their full world pose at the same cadence.

### Retargeting guidance for a robotic hand
1. **Align coordinate frames** – ensure the robot controller expects Y-up, +Z forward (or convert the MocapApi poses into your
   robot base frame) before consuming the wrist transforms.
2. **Scale and offset** – use the wrist 6D pose as the controller target for each arm. If your robot uses different neutral
   offsets, add them after transforming the pose into the robot frame.
3. **Map ergonomic angles to actuators** – for each robot finger joint, choose the corresponding MocapApi joint and apply the
   flexion/abduction/twist values. Clamp or scale to the robot’s mechanical limits; for coupled joints, blend multiple human
   joints to drive a single actuator. Update the mapping logic in `HandRetargeter::Retarget`
   inside `src/teleop_main.cpp` to change how human finger motion drives your
   dexterous hand.
4. **Use raw data when needed** – if your robot expects absolute transforms instead of ergonomic angles, feed it the raw
   world-space finger poses from `TeleopMapping::RawFingerPoses()` and compute your own IK or synergy mapping.

## URDF visualization (Python)

The repository includes a lightweight PyBullet script to inspect any URDF in the `urdf/` folder.

```bash
pip install -r requirements.txt
python scripts/visualize_urdf.py --urdf urdf/lrmate_with_unijoint_hand.urdf
```

Useful flags:
- `--free-base` – allow the base to move (defaults to fixed base)
- `--no-plane` – disable the ground plane
- `--realtime` – run PyBullet in realtime instead of fixed stepping

## Skeleton scaling and body-length data

You do **not** need to supply custom body-length parameters when pulling the MocapApi skeleton. Each avatar already carries the segment lengths through the joints' default local positions and bind pose that you can query from `IMCPJoint`/`IMCPBodyPart`. If you want to inspect or log those values, fetch the joint list from `IMCPAvatar`, then read the default local positions for each handle (e.g., via `GetJointDefaultLocalPosition`) to derive per-bone lengths; the BVH stream viewer already captures these local positions when it builds `JointSample` structures.
