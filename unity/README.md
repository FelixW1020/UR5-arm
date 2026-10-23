# Quest 3 side: Robot ↔ VR registration (Unity)

These scripts are what a Unity game needs to share coordinates with the robot: register the headset's world to the UR5, stream the headset and controllers into ROS, and place robot-side objects in VR. Drop the `RobotRegistration/` folder into your project's `Assets/`.

> **Status:** these scripts have **not** run on a headset yet. They were written against ROS-TCP-Connector v0.7.0, Meta XR Core SDK and MR Utility Kit (MRUK) v81–v207 without a Unity install. They do compile (C# 9, with and without the `ROS2` define, MRUK v81 and v207 API surfaces) against stubs of the Unity/Meta APIs and the real ROS-TCP-Connector message classes generated from `src/vr_interfaces`. The messages they build were round-tripped through `rclpy`, and a passthrough frame built this way registers correctly through the real `vr_tag_detector` and `vr_registration`. Expect small fixes on the first import, and run the camera checklist below once.

## How the coordinates work

```
base_link (robot) ──[vr_registration, static TF]──► vr_world ══ RobotWorldFrame (a GameObject in Unity)
                                                       └─[vr_tracking/quest_bridge, TF]──► quest_head, quest_left_controller, quest_right_controller
```

- `RobotWorldFrame` is an empty GameObject you place once in the room. Every pose sent to or received from ROS is relative to it.
- **Recentering the headset moves it.** Holding the Meta button (or *Reset View*) shifts Unity's whole world in the room, `RobotWorldFrame` included, so the registration no longer holds. `RobotWorldFrame` notices the recenter and sets `RegistrationStale`. Until the next successful registration:
  - `QuestStatePublisher` sends everything as untracked, so nothing on the robot side follows wrong poses;
  - the registration scripts drop the sightings, touches or samples collected before the recenter, and say so in their status.

  Two things keep `vr_world` fixed through a recenter: the *Stage* tracking origin (recentering off, needs a boundary), or a localized `OVRSpatialAnchor` on `RobotWorldFrame` that you save and reload yourself.
- Coordinates are converted with ROS-TCP-Connector's `FLU` helpers (x forward, y left, z up, the ROS convention). The ROS side never sees Unity's left-handed Y-up axes. An object's Unity forward becomes ROS x: `quest_head` x is the gaze direction, a controller's x is its pointing direction.
- After registration, ROS republishes poses in `vr_world`. `RosPoseFollower` places objects from those topics:
  - `/vr/robot_base_pose`: where to put the UR5 model.
  - `/vr/object_localization/target_pose`: the cup.
  - The arm itself animates from `/joint_states`.

## Scripts

| Script | Does | ROS interface |
|---|---|---|
| `RobotWorldFrame` | Defines `vr_world`; Unity ↔ ROS pose conversion | — |
| `QuestStatePublisher` | Streams headset + both controllers (poses, tracked flags, triggers, stick, buttons) | pub `/quest/state` (`vr_interfaces/QuestState`), 30 Hz |
| `PassthroughTagRegistration` | Register by **looking** at the tags (UI + buttons) | calls `/vr_tag_detector/register`, `/vr_tag_detector/reset` |
| `PassthroughFrameStreamer` | Sends passthrough camera frames while enabled | pub `/quest/passthrough` (`vr_interfaces/PassthroughFrame`), 3 Hz |
| `PassthroughCameraSource` | The only code that touches Meta's Passthrough Camera API (adapter) | — |
| `TagTouchRegistration` | Register by touching tag centres with the controller tip | calls `/vr_registration/register_tags` |
| `ControllerFlangeRegistration` | Register with a controller clamped to the flange | calls `/vr_registration/add_controller_sample`, `/vr_registration/solve_controller` |
| `TagMapVisualizer` | Draws the robot's tag map in VR (visual check) | calls `/vr_registration/get_tag_map` |
| `RosPoseFollower` | Moves a GameObject to a `PoseStamped` in `vr_world` | sub e.g. `/vr/robot_base_pose` |
| `RosUtil` | Stamps, service calls with a timeout, the empty-request fix (below) | — |

## Project setup

1. **Unity and Meta SDK:**
   - Unity 2022.3 LTS works with MRUK v81–v201. MRUK v203 and newer need Unity 6 (6000.0.66f2+). Meta lists 2022.3.58f1 and 6000.0.38f1 as tested.
   - Install the Meta XR Core SDK and **Meta MR Utility Kit (MRUK) v81 or newer**. MRUK provides `PassthroughCameraAccess`; v78 and older don't have it. Only `PassthroughCameraSource` uses MRUK.
   - Switch the platform to Android and run *Meta > Tools > Project Setup Tool* (fix all).
2. **ROS-TCP-Connector:** in Package Manager, choose *Add package from git URL*:
   `https://github.com/Unity-Technologies/ROS-TCP-Connector.git?path=/com.unity.robotics.ros-tcp-connector`
3. **Point it at the ROS PC:** *Robotics > ROS Settings*: Protocol **ROS2**, ROS IP = the robot PC's LAN address, port 10000. Under *Player Settings > Android*, allow internet access.
   - Choose ROS2 **while Android is the active platform**: the setting is stored as the `ROS2` scripting define of the current build target only.
   - Check that *Player Settings > Android > Scripting Define Symbols* contains `ROS2`. Without it, every header gets ROS 1's extra `seq` field and nothing deserializes on the robot PC.
4. **Generate the C# message classes** (*Robotics > Generate ROS Messages…*). Browse to this repo's `src/vr_interfaces` and build all of its msg and srv files. That creates `RosMessageTypes.VrInterfaces`.
   - **Regenerate after pulling.** `TagObservation` gained `has_orientation`/`orientation`, and `ControllerState`, `QuestState` and `PassthroughFrame` are new. Delete the old `Assets/RosMessages/VrInterfaces` folder first, so no stale class survives.
   - Don't generate `std_msgs`, `geometry_msgs`, `sensor_msgs` (`CompressedImage`, `CameraInfo`) or `builtin_interfaces`. They're built into ROS-TCP-Connector, and duplicates won't compile.
   - `std_srvs/Trigger` is *not* built in, but you don't need to generate it either. `RosUtil.cs` has a small hand-written `TriggerResult` for the one place that uses it.
5. **ROS side:** start the endpoint from a shell where this workspace is sourced. It has to import `vr_interfaces` to (de)serialize the new messages. `vr.launch.py` starts it on port 10000.

### Scene

- An `OVRCameraRig` with Tracking Origin Type set to *Floor Level*. On its OVR Manager, set Passthrough Support to *Supported* (or *Required*) and tick *Enable Passthrough*.
- An empty `RobotWorldFrame`. Optionally add an `OVRSpatialAnchor` to it and save/reload it yourself. While the anchor is localized, recentering doesn't invalidate the registration (see above).
- `QuestStatePublisher` on any GameObject. It finds the rig itself.
- A `TagMapVisualizer`. Wire each registration script's `onRegistered` to `TagMapVisualizer.Refresh`.
- Pick at least one way to register:
  - **Passthrough tags:** a GameObject with `PassthroughCameraAccess` (MRUK; Camera Position *Left*, Requested Resolution 1280×960, **component unticked**), `PassthroughCameraSource`, `PassthroughFrameStreamer` (unticked) and `PassthroughTagRegistration`. Optionally add a `TextMesh` as `status`. See the next section.
  - **Tag touch:** a `ControllerTip` child under `RightControllerAnchor`, with a 1 cm sphere on it. Adjust its local position in passthrough until the sphere sits on the controller's physical tip. Then add `TagTouchRegistration`.
  - **Flange:** `ControllerFlangeRegistration`, with `RightControllerAnchor` as the clamped controller.
- `PassthroughTagRegistration` and `TagTouchRegistration` both use A and B on the right controller, so **enable only one of them at a time**. `PassthroughTagRegistration`'s buttons are configurable in the inspector.
- **Robot model (for the game):** import the UR5 URDF with Unity's URDF Importer, put `RosPoseFollower` (topic `/vr/robot_base_pose`) on its root, and drive the joints from `/joint_states`.

### Passthrough Camera API (PCA) setup

- **Requirements:** Quest 3 or 3S on **Horizon OS v74+** (v83+ adds 1280×1280), with passthrough enabled.
- **Testing from the Editor over Link:** MRUK v207 needs Meta Horizon Link v85+ and a headset on Horizon OS v85+.
- **Manifest permission:** on the OVR Manager, tick **Enable Passthrough Camera Access** (the Project Setup Tool item "Passthrough Camera Access requires…" does this). The build then adds `<uses-permission android:name="horizonos.permission.HEADSET_CAMERA" />` to the manifest. Check it with *Window > Meta > Tools > Android Manifest Tool*.
- **Runtime permission:** `PassthroughCameraSource` asks for it the first time streaming starts (`requestPermission`). Meta says to request permissions from one place only: if you tick OVR Manager's *Permission Requests On Startup > Passthrough Camera Access* instead, untick `requestPermission`.
  - If the user denies the permission, the status line says so. Allow it again in the headset's app permission settings.
- **Camera states:** the status line shows every not-ready state as text ("Waiting for the headset camera permission", "Starting the passthrough camera…", "Hold still…"). Frames are only sent once the camera is playing.
- **Keep the PCA component unticked in the scene.** `PassthroughFrameStreamer` switches the camera on only while streaming, then off again (`stopCameraWhenIdle`).
- **If an SDK update breaks the camera code,** the fix belongs in `PassthroughCameraSource.cs` only.

## Registering

On the robot PC, run `ros2 launch ur5_bringup vr.launch.py ...` (see the main README). Then, in the headset, use one of these:

| Method | Steps | Expected accuracy |
|---|---|---|
| **Passthrough tags** (`PassthroughTagRegistration`) | Enable it: frames stream and old sightings are cleared. Look at each tag from 0.4–1.2 m and hold your head still for ~2 s per tag (frames are skipped while you turn faster than 10°/s). A registers, B starts over. One tag works; 2–4 tags spread over the table are better. | about 1–2 cm and ~1° (with 2–4 tags; one tag's heading comes only from its own orientation, so expect more) |
| **Tag touch** (`TagTouchRegistration`) | Touch each tag's centre with the controller tip and pull the trigger. B undoes, A registers. Use at least 3 tags spread across the table. | about 1 cm (depends on tip-offset care) |
| **Controller on flange** (`ControllerFlangeRegistration`) | Clamp the right controller to the tool flange. At 10–15 varied, still arm poses, press X; then press Y. | a few mm |

The passthrough reply lists every tag as `tag <id>: n=<frames>, spread <mm>, residual <mm>`:

- **spread** is how much that tag's position moved between frames.
- **residual** is how far it lands from the robot's tag map after registration.

The passthrough accuracy is limited by the 1280×960 camera (a 10 cm tag at 1 m is ~85 px wide), the headset's own tracking, and the factory camera-to-head calibration.

**Check:** with passthrough on, `TagMapVisualizer`'s orange squares should land on the printed tags. Redo registration after a recenter, if you move `RobotWorldFrame`, if your spatial anchor didn't restore, or whenever the squares are off the tags (for example after an app restart).

## Verify the camera conventions (once, after the first build)

`PassthroughFrameStreamer` promises three things, and nothing on a headset has checked them yet:

- an upright JPEG;
- `K` in OpenCV pixels for exactly that JPEG;
- the camera pose at capture time, x = viewing direction, y = image-left, z = image-up.

1. **Frames arrive:** run `ros2 topic hz /quest/passthrough` (~3 Hz while streaming). The Unity log prints `[PassthroughCameraSource] K for 1280x960: fx … cy … (principal point … px from the centre)`; expect fx ≈ fy and the principal point within a few % of the centre.
2. **Picture:** open `/vr_tag_detector/debug_image` in `rqt_image_view` or RViz.
   - It must look like what you see: ceiling up, not mirrored.
   - If it's upside down, tick `flipVertically` on `PassthroughFrameStreamer`. Only the pixels change; K and the pose already describe the upright image.
   - Tags get green outlines and axes. Red is x (toward the printed tag's right), green is y (toward its top), and blue z points out of the paper.
3. **Pose:** look at one tag from ~0.6 m. Slowly turn your head left/right and up/down, keeping the tag in view, then press A. The tag's spread should be a few mm (≤ 10 mm).
   - A spread that grows with head **rotation** means the pose or K is wrong.
   - If it mostly grows when you tilt your head **up/down**, and the logged cy is far (> ~10 px) from the centre, tick `principalPointFromTop` on `PassthroughCameraSource`, press B and try again. Keep whichever setting gives the smaller spread.
4. **Registration:** the reply's residuals should be ≤ ~15 mm. After registration:
   - `TagMapVisualizer`'s squares should sit on the printed tags, within ~1–2 cm, with the edges parallel.
   - In RViz, the fused tags (`/vr_tag_detector/markers`) should overlap the robot's `tag_<id>` frames.
5. **Cross-check:** touch a tag centre with the controller tip (`TagTouchRegistration`'s sphere). It should be within ~2 cm of where the passthrough registration puts it.

## Streaming the headset and controllers

`QuestStatePublisher` sends `/quest/state` at `rateHz` (30 Hz). On the ROS side `vr_tracking`'s `quest_bridge` turns it into what robot code consumes:

- TF `vr_world → quest_head / quest_left_controller / quest_right_controller`.
- `PoseStamped` topics in `base_link`, once registered.
- `sensor_msgs/Joy` per controller.

The fields per controller:

- **`tracked`:** from `OVRInput.GetControllerPositionTracked`, and false for everything while `RobotWorldFrame.RegistrationStale` is set (after a recenter).
- **Pose:** the controller anchor.
- **Analog:** `index_trigger`, `hand_trigger` (grip), and the thumbstick x/y.
- **Buttons:** `button_primary` (A / X), `button_secondary` (B / Y), `thumbstick_pressed`, and `menu` (left only).

Inputs are read per controller (`Controller.LTouch` / `RTouch`). With a single controller named, OVRInput maps X/A to `Button.One` and Y/B to `Button.Two`; `Button.Three`/`Four` only exist on the combined `Controller.Touch`. (This was a bug in `ControllerFlangeRegistration`, which read X/Y as Three/Four on `LTouch` and so never fired.)

## Notes

- **Empty service requests:** ROS 2 serialises an empty message as one dummy byte, but ROS-TCP-Connector's generated empty requests (e.g. `GetTagMapRequest`) serialise as nothing. The endpoint then fails to deserialize them, under both Fast-DDS and Cyclone, and never replies. So `TagMapVisualizer` and the reset call send `RosUtil`'s `EmptyRequest` instead.
- **Service timeouts:** the endpoint also never replies when a service isn't running. All service calls therefore time out (`serviceTimeoutS`) and say so, instead of locking the buttons.
- **Topics and services are registered by message name** (`RegisterPublisher(topic, XxxMsg.k_RosMessageName)`), never with the generic `RegisterPublisher<XxxMsg>`. In a player build the generated classes register their names after the first scene's `OnEnable`. A generic call made from `OnEnable` would therefore announce the topic to the endpoint with no type, and it would never work on the headset, even though it works in the Editor.
- **Not a Store app:** PCA and the scripts here are for a lab app.
- **Alternative tag source:** MRUK's QR-code trackables (`MRUKTrackable`, experimental: not accepted on the Store, needs the Spatial Data permission) give the OS's own pose of a printed QR code, without streaming images. The robot cameras' `aruco_tag_mapper` maps ArUco tags, not QR codes, so using them would need a QR detector on the ROS side as well.
