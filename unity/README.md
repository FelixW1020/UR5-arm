# Quest 3 side: Robot ↔ VR registration (Unity)

These scripts are what a Unity game needs to share coordinates with the robot. Drop the `RobotRegistration/` folder into your project's `Assets/`.

> **Status:** these scripts have **not** been compiled or run yet. They were written against ROS-TCP-Connector and Meta XR Core SDK APIs without a Unity install. Expect small fixes on the first import.

## How the coordinates work

```
base_link (robot) ──[vr_registration, static TF]──► vr_world ══ RobotWorldFrame (a GameObject in Unity)
```

- `RobotWorldFrame` is an empty GameObject you place once in the room. Every pose sent to or received from ROS is relative to it, so recentering the headset doesn't break the alignment.
- Coordinates are converted with ROS-TCP-Connector's `FLU` helpers (x forward, y left, z up, the ROS convention). The ROS side never sees Unity's left-handed Y-up axes.
- After registration, ROS republishes poses in `vr_world`. `RosPoseFollower` places objects from those topics:
  - `/vr/robot_base_pose`: where to put the UR5 model.
  - `/vr/object_localization/target_pose`: the cup.
  - The arm itself animates from `/joint_states`.

## Project setup

1. **Unity 2022.3 LTS or newer.** Install the Meta XR Core SDK (Meta XR All-in-One), switch the platform to Android, and run *Meta > Tools > Project Setup Tool* (fix all).
2. **ROS-TCP-Connector:** in Package Manager, choose *Add package from git URL*:
   `https://github.com/Unity-Technologies/ROS-TCP-Connector.git?path=/com.unity.robotics.ros-tcp-connector`
3. **Generate the C# message classes:** *Robotics > Generate ROS Messages…*, browse to this repo's `src/vr_interfaces`, and build its msg and srv files. This creates the `RosMessageTypes.VrInterfaces` namespace the scripts use.
4. **Point it at the ROS PC:** *Robotics > ROS Settings*: Protocol **ROS2**, ROS IP = the robot PC's LAN address, port 10000. Under *Player Settings > Android*, allow internet access.
5. **Scene:**
   - An `OVRCameraRig` with Tracking Origin Type set to *Floor Level*. Enable passthrough if you want mixed reality.
   - An empty `RobotWorldFrame`. Optionally add an `OVRSpatialAnchor` to it and persist it.
   - A `ControllerTip` child under `RightControllerAnchor`, with a 1 cm sphere on it. Adjust its local position in passthrough until the sphere sits on the controller's physical tip.
   - `TagTouchRegistration` and/or `ControllerFlangeRegistration`, plus `TagMapVisualizer`. Wire `onRegistered` to `TagMapVisualizer.Refresh`.
6. **Robot model (for the game):** import the UR5 URDF with Unity's URDF Importer, put `RosPoseFollower` (topic `/vr/robot_base_pose`) on its root, and drive the joints from `/joint_states`.

## Registering

On the robot PC, run `ros2 launch ur5_bringup vr.launch.py ...` (see the main README). Then, in the headset, use one of these:

| Method | Steps | Expected accuracy |
|---|---|---|
| **Tag touch** (`TagTouchRegistration`) | Touch each tag's centre with the controller tip and pull the trigger. B undoes, A registers. Use at least 3 tags spread across the table. | about 1 cm (depends on tip-offset care) |
| **Controller on flange** (`ControllerFlangeRegistration`) | Clamp the right controller to the tool flange. At 10–15 varied, still arm poses, press X; then press Y. | a few mm |

**Check:** with passthrough on, `TagMapVisualizer`'s orange squares should land on the printed tags. Redo registration if you move `RobotWorldFrame`, or if your spatial anchor didn't restore.

## Not implemented yet: automatic tag detection with the passthrough cameras

Quest 3 exposes its passthrough camera images to apps through Meta's Passthrough Camera API. An ArUco detector running on those images (e.g. OpenCV for Unity) can call `TagTouchRegistration.AddObservation(id, worldPosition)` for each tag centre, then `Submit()`. Everything on the ROS side already supports it. Expect cm-level accuracy, because the passthrough images are low resolution.
