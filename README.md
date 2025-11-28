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

## Skeleton scaling and body-length data

You do **not** need to supply custom body-length parameters when pulling the MocapApi skeleton. Each avatar already carries the segment lengths through the joints' default local positions and bind pose that you can query from `IMCPJoint`/`IMCPBodyPart`. If you want to inspect or log those values, fetch the joint list from `IMCPAvatar`, then read the default local positions for each handle (e.g., via `GetJointDefaultLocalPosition`) to derive per-bone lengths; the BVH stream viewer already captures these local positions when it builds `JointSample` structures.
