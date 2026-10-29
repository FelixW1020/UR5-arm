# Unity Developer Guide: Robot Calibration in Your Game

The scripts in [`RobotRegistration/`](RobotRegistration) connect a Quest 3 Unity game to the UR5's ROS computer. After a short calibration (the player looks at a printed tag and presses a button), your game knows exactly where the real robot is, and the robot side knows where the headset and controllers are.

You can **build the game first and add calibration later**. Follow the [five rules](#build-the-game-now-add-calibration-later) from day one, and adding it later is a drop-in job.

> **Status:** these scripts compile, and the messages they build were tested against the real ROS nodes, but they haven't run on a headset yet. Expect small fixes on the first import, and run the [first-build checklist](#first-build-checklist) once.

## How it works

- **`RobotWorldFrame`** is an empty GameObject that stands for the headset's world (`vr_world` on the ROS side). Every position sent to or received from ROS is relative to it.
- **Calibration** (called *registration* in the code): the headset's passthrough camera looks at a printed ArUco tag whose position relative to the robot is already known. The ROS computer then works out where the robot is in `vr_world`.
- **`RobotBase`** is a GameObject with `RosPoseFollower` on it. It moves onto the real robot's base once calibration succeeds, so **anything you parent under it lines up with the real robot**. Robot coordinates use the robot's own axes (`base_link`): x and y lie flat on the table (the red and green arrows of `base_link` in RViz), and z is up.
- **`QuestStatePublisher`** streams the headset and controllers to ROS, so robot code can use them (see the [main README](../README.md#use-the-headset-in-robot-code)).

```
Quest app (Unity)                                   ROS PC
──────────────────                                  ──────
PassthroughTagRegistration ── camera frames ──────► finds the tag, solves the calibration
QuestStatePublisher ───────── headset + controllers ► /quest/... topics for robot code
RobotBase (RosPoseFollower) ◄── /vr/robot_base_pose ─ where the robot is in vr_world
```

## Build the game now, add calibration later

Five rules keep that easy:

1. **The player walks; the rig never moves.** Don't add teleport or smooth locomotion, and don't move or rotate the `OVRCameraRig` from code. Moving the rig slides the whole virtual world away from the real robot.
2. **One `RobotWorldFrame` for the whole game.** Keep it, `RobotBase`, the `OVRCameraRig`, the calibration objects and your calibration script in a scene that stays loaded, and load levels additively (or make them root objects and mark them `DontDestroyOnLoad`). Inspector references can't point into another scene, so scripts in level scenes use `RobotWorldFrame.Instance`. Never move `RobotWorldFrame` after calibrating.
3. **Robot things go under `RobotBase`.** Make an empty `RobotBase` GameObject and parent everything that belongs to the robot to it: the robot model, targets on the table, effects around the arm. Level scenes can't parent to it in the Editor, so attach level content at runtime (`Instantiate(prefab, robotBase)`). Until calibration is added, place `RobotBase` by hand where the robot roughly is. After calibration it jumps to the real spot by itself.
4. **Plan a calibration moment.** The game should start with a short "look at the tag" step, and be able to return to it (for example, from the pause menu).
5. **Mixed reality settings:** passthrough visible (see the `OVRCameraRig` row [below](#2-add-these-objects-to-your-scene)), and Tracking Origin Type set to *Floor Level*.

During calibration the right controller's **A** and **B** buttons are used, unless you [drive calibration from your own UI](#4-connect-it-to-your-game-flow).

## Add calibration to your game

### 1. Project setup (once)

1. **Unity and Meta SDK:**
   - Unity 2022.3 LTS works with MR Utility Kit (MRUK) v81–v201. MRUK v203 and newer need Unity 6 (6000.0.66f2+).
   - Install the **Meta XR Core SDK** and **MRUK v81 or newer** (v78 and older lack the camera API used here).
   - Install the **OpenXR Plugin** (`com.unity.xr.openxr`), which Meta recommends from SDK v74. In *Project Settings > XR Plug-in Management*, Android tab, tick **OpenXR** and the **Meta XR** feature group, and add the **Oculus Touch Controller Profile** under OpenXR's interaction profiles.
   - Switch the platform to Android and run *Window > Meta > Tools > Project Setup Tool* (fix all; older SDKs have it under *Meta > Tools*).
   - Tested on a Quest 3 with Unity 6000.0.85f1, Meta XR Core SDK and MRUK 207.0.1, and OpenXR Plugin 1.18.0.
2. **ROS-TCP-Connector:** in Package Manager, choose *Add package from git URL*:
   `https://github.com/Unity-Technologies/ROS-TCP-Connector.git?path=/com.unity.robotics.ros-tcp-connector#v0.7.0`
3. **Point it at the ROS PC:** *Robotics > ROS Settings*: Protocol **ROS2**, ROS IP = the ROS PC's address on your network, port 10000. Under *Player Settings > Android*, allow internet access.
   - Choose ROS2 **while Android is the active platform**: the setting is saved as the `ROS2` scripting define of the current platform only.
   - Check that *Player Settings > Android > Scripting Define Symbols* contains `ROS2`. Without it nothing deserializes on the ROS PC.
   - Get the IP from whoever runs the ROS PC (`hostname -I` on that PC). It's built into the app, so rebuild if it changes.
   - The Quest and the PC must be on the same network, and it must let devices talk to each other (campus Wi-Fi often doesn't; a lab router does). When the app connects, the ROS terminal logs `Connection from <Quest IP>`. If it doesn't, a "No reply from …" message in the headset means a network problem, not a ROS one.
4. **Generate the message classes:** *Robotics > Generate ROS Messages…*, browse to this repo's `src/vr_interfaces`, and build all of its msg and srv files.
   - Don't generate `std_msgs`, `geometry_msgs`, `sensor_msgs` or `builtin_interfaces`. They're built into ROS-TCP-Connector, and duplicates won't compile.
   - When `src/vr_interfaces` changes, delete `Assets/RosMessages/VrInterfaces` and generate again.
5. **Copy the scripts:** copy the `RobotRegistration/` folder into your project's `Assets/`.

### 2. Add these objects to your scene

| GameObject | Components | Settings |
|---|---|---|
| `OVRCameraRig` (you already have it) | OVR Manager, OVR Passthrough Layer | OVR Manager: Tracking Origin Type *Floor Level*. Passthrough Support *Supported*, tick *Enable Passthrough* and *Enable Passthrough Camera Access* (a project-wide setting, greyed out until Passthrough Support is set). To actually see the room: an `OVRPassthroughLayer` (always an underlay in SDK v207; older SDKs: Placement *Underlay*), and the `CenterEyeAnchor` camera's background set to a solid colour with alpha 0. Meta's *Passthrough* building block (*Window > Meta > Tools > Building Blocks*) does both |
| `RobotWorldFrame` (empty) | `RobotWorldFrame` | Anywhere, but don't move it after calibrating. Only one per game |
| `QuestState` (empty) | `QuestStatePublisher` | It finds the rig by itself |
| `Calibration` | `PassthroughCameraAccess` (from MRUK), `PassthroughCameraSource`, `PassthroughFrameStreamer`, `PassthroughTagRegistration` | `PassthroughCameraAccess`: Camera Position *Left*, Requested Resolution 1280×960, **component unticked**. `PassthroughFrameStreamer`: **unticked**. Optional: a `TextMesh` child as the `status` label |
| `TagMap` (empty) | `TagMapVisualizer` | On `RobotWorldFrame`, add `TagMapVisualizer.Refresh` to its *On Registered* event |
| `RobotBase` (empty) | `RosPoseFollower` | Topic `/vr/robot_base_pose`. Tick `waitForRegistration`: otherwise it first jumps to where the ROS PC's last saved calibration puts the robot, which may be off in a new session |

The two unticked components are switched on by the scripts only while the camera is needed.

**Your game already uses the passthrough camera** (its own `PassthroughCameraAccess`, or Meta's *Passthrough Camera Access* building block)? MRUK allows only one per camera. Either set the calibration's Camera Position to *Right*, or delete the calibration's `PassthroughCameraAccess`, drag your existing one into `PassthroughCameraSource`'s *Camera Access* field, and untick `stopCameraWhenIdle` so calibration doesn't switch your camera off.

### 3. Try a calibration

1. On the ROS PC, start `ros2 launch ur5_bringup vr.launch.py ...` (see the [main README](../README.md#step-3-register-the-headset-every-session)), or the simulation (see [Testing without the real robot](#6-testing-without-the-real-robot)).
2. Run the game. The status label says what the camera is doing.
3. Look at a tag from 0.4–1.2 m away and hold your head still for ~2 s.
4. Press **A**. The reply lists each tag with its *spread* (how much it moved between frames) and *residual* (how far it lands from where the robot thinks it is). **B** starts over.
5. **Check:** with passthrough on, `TagMapVisualizer`'s squares should sit on the printed tags.

One tag works. 2–4 tags spread over the table are better: about 1–2 cm and ~1°.

### 4. Connect it to your game flow

`RobotWorldFrame` tells your game when the calibration holds:

- `IsRegistered` is true after a successful calibration in this session, and false again after a recenter.
- `onRegistered` fires after every successful calibration.
- `onRegistrationLost` fires when a headset recenter breaks the calibration.

A script that calibrates at every start and pauses when the calibration breaks:

```csharp
using UnityEngine;

public class CalibrationFlow : MonoBehaviour
{
    public RobotWorldFrame robotWorld; // empty = RobotWorldFrame.Instance
    public GameObject calibration;     // the GameObject with PassthroughTagRegistration
    public bool skipCalibration;       // tick to test the game without the ROS PC

    void Start()
    {
        if (robotWorld == null) robotWorld = RobotWorldFrame.Instance;
        if (skipCalibration) { OnCalibrated(); return; }
        robotWorld.onRegistered.AddListener(OnCalibrated);
        robotWorld.onRegistrationLost.AddListener(OnCalibrationLost);
        calibration.SetActive(true);
    }

    void OnDestroy()
    {
        if (robotWorld == null) return;
        robotWorld.onRegistered.RemoveListener(OnCalibrated);
        robotWorld.onRegistrationLost.RemoveListener(OnCalibrationLost);
    }

    void OnCalibrated()
    {
        calibration.SetActive(false);  // camera off, A and B free for the game
        // start or resume the game here
    }

    void OnCalibrationLost()
    {
        // pause the game here
        calibration.SetActive(true);   // forgets the old sightings and starts the camera again
    }
}
```

- **`RobotBase` arrives a moment later.** `onRegistered` fires up to ~0.5 s before `RobotBase` reaches its new spot, and it glides there. Parent things under `RobotBase` instead of copying its position when the event fires.
- **Your own buttons instead of A/B:** untick `useControllerButtons` on `PassthroughTagRegistration`, and call its `Register()` (and `ResetSightings()` to start over) from your own UI. `IsBusy` is true while a call is waiting, and `StatusText` holds the message to show the player.
- **Calibrate from the headset.** A register call made on the ROS PC works on the robot side, but the app never hears about it, so `onRegistered` doesn't fire.
- **Calibrate every session.** The ROS PC reloads the last calibration on start-up, but the headset's world can be different after the app restarts, so `IsRegistered` starts false. If you let players skip calibration, check that the tag squares still sit on the printed tags.

### 5. Use robot coordinates in game code

ROS and Unity use different axes. ROS: x forward, y left, z up. Unity: x right, y up, z forward. Both use metres. For the robot, "forward" and "left" mean `base_link`'s x and y, so check which way they point in RViz. ROS-TCP-Connector converts between the two:

```csharp
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

public class RobotCoordinatesExample : MonoBehaviour
{
    public Transform robotBase;        // the RobotBase GameObject
    public Transform target;           // something to place next to the robot
    public Transform rightHand;        // OVRCameraRig > TrackingSpace > RightHandAnchor > RightControllerAnchor

    void Start()
    {
        // 40 cm along the robot's x axis, 10 cm along its y axis, 5 cm up:
        target.SetParent(robotBase, false);
        target.localPosition = new Vector3<FLU>(0.40f, 0.10f, 0.05f).toUnity;
    }

    void Update()
    {
        // Where is the player's right hand, in robot coordinates?
        Vector3<FLU> hand = robotBase.InverseTransformPoint(rightHand.position).To<FLU>();
        Debug.Log($"hand at x={hand.x:F2} y={hand.y:F2} z={hand.z:F2} m from the robot base");
    }
}
```

Other things you can place the same way:

- **The cup** found by the robot's cameras: another `RosPoseFollower` with topic `/vr/object_localization/target_pose`.
- **A moving robot model:** import the UR5 URDF with Unity's URDF Importer as a child of `RobotBase`, and drive its joints from `/joint_states` (not included here).

### 6. Testing without the real robot

- **No ROS at all:** calibration can't succeed, so tick `skipCalibration` on `CalibrationFlow`. `RobotBase` stays where you placed it by hand.
- **Real headset, simulated robot:** print one tag (`ros2 run aruco_tag_mapper generate_tags --ids 0`) and tape it to a table. Pick a spot on the table to stand in for the robot's base, and a direction for its +x (+y is 90° to the left of it). Write the tag map as in [Measure the tag by hand](../README.md#no-robot-cameras-measure-the-tag-by-hand), measuring from that spot. Then run `ros2 launch ur5_bringup vr_sim.launch.py quest:=real` on a ROS PC. The game calibrates against the real tag, and `RobotBase` lands on the spot you picked, where RViz draws the simulated UR5. Hold the right controller's grip to move the simulated arm in RViz.

## Reference

### Scripts

| Script | Does | ROS interface |
|---|---|---|
| `RobotWorldFrame` | Defines `vr_world`, converts poses between Unity and ROS, tracks whether the calibration holds | — |
| `QuestStatePublisher` | Streams the headset and both controllers (poses, tracked flags, triggers, stick, buttons) | publishes `/quest/state` (`vr_interfaces/QuestState`), 30 Hz |
| `PassthroughTagRegistration` | Calibrate by **looking** at the tags (status text + buttons) | calls `/vr_tag_detector/register`, `/vr_tag_detector/reset` |
| `PassthroughFrameStreamer` | Sends passthrough camera frames while enabled | publishes `/quest/passthrough` (`vr_interfaces/PassthroughFrame`), 3 Hz |
| `PassthroughCameraSource` | The only code that touches Meta's Passthrough Camera API | — |
| `TagTouchRegistration` | Calibrate by touching tag centres with the controller tip | calls `/vr_registration/register_tags` |
| `ControllerFlangeRegistration` | Calibrate with a controller clamped to the robot's flange | calls `/vr_registration/add_controller_sample`, `/vr_registration/solve_controller`, `/vr_registration/reset_controller_samples` |
| `TagMapVisualizer` | Draws the robot's tag map in VR (a visual check) | calls `/vr_registration/get_tag_map` |
| `RosPoseFollower` | Moves a GameObject to a `PoseStamped` in `vr_world` (optionally only once calibrated: `waitForRegistration`) | subscribes, e.g. `/vr/robot_base_pose` |
| `RosUtil` | Timestamps, service calls with a timeout, the empty-request fix (see Notes) | — |

### Other ways to register

| Method | Steps | Expected accuracy |
|---|---|---|
| **Passthrough tags** (`PassthroughTagRegistration`, the default above) | Look at the tags, press A | about 1–2 cm and ~1° with 2–4 tags (one tag's heading comes only from its own orientation, so expect more) |
| **Tag touch** (`TagTouchRegistration`) | Add a `ControllerTip` child under `RightControllerAnchor` with a 1 cm sphere on it, and move it in passthrough until the sphere sits on the controller's physical tip. Then touch each tag's centre with the tip and pull the trigger. B undoes, A registers. Use at least 3 tags spread across the table | about 1 cm (depends on how carefully the tip is placed) |
| **Controller on flange** (`ControllerFlangeRegistration`, with `RightControllerAnchor` as the clamped controller) | Clamp the right controller to the tool flange. At 10–15 varied, still arm poses press X, then press Y | a few mm |

`PassthroughTagRegistration` and `TagTouchRegistration` both use A and B on the right controller, so **enable only one of them at a time**.

### Recentering

Holding the Meta button (or choosing *Reset View*) shifts Unity's whole world in the room, `RobotWorldFrame` included, so the calibration no longer holds. `RobotWorldFrame` notices the recenter, and the registration scripts drop the sightings, touches or samples collected before it. If the headset was already calibrated in this session, `RobotWorldFrame` also sets `RegistrationStale`, makes `IsRegistered` false and fires `onRegistrationLost`. Until the next successful calibration, `QuestStatePublisher` then sends everything as untracked, so the robot side doesn't follow wrong poses.

Two things keep `vr_world` fixed through a recenter: the *Stage* tracking origin (recentering off, needs a boundary), or a localized `OVRSpatialAnchor` on `RobotWorldFrame` that you save and reload yourself.

### Passthrough Camera API setup

- **Requirements:** Quest 3 or 3S on **Horizon OS v74+** (v83+ adds 1280×1280), with passthrough enabled.
- **Testing from the Editor over Link:** MRUK v207 needs Meta Horizon Link v85+ and a headset on Horizon OS v85+.
- **Manifest permission:** ticking *Enable Passthrough Camera Access* on the OVR Manager adds `horizonos.permission.HEADSET_CAMERA` to the manifest when the app is built. Check the built app: `aapt dump permissions <your>.apk` (`aapt` is in the Android SDK's `build-tools`).
- **Runtime permission:** `PassthroughCameraSource` asks for it the first time the camera starts (`requestPermission`). Meta says to request permissions from one place only: if you tick OVR Manager's *Permission Requests On Startup > Passthrough Camera Access* instead, untick `requestPermission`. If the player denies it, the status says so; allow it again in the headset's app permission settings.
- **Keep the `PassthroughCameraAccess` component unticked in the scene.** `PassthroughFrameStreamer` switches the camera on only while calibrating, and off again afterwards (`stopCameraWhenIdle`).
- **If an SDK update breaks the camera code,** the fix belongs in `PassthroughCameraSource.cs` only.

### First-build checklist

`PassthroughFrameStreamer` promises three things that no headset has checked yet: an upright JPEG, the camera intrinsics `K` (in OpenCV pixels) for exactly that JPEG, and the camera pose at capture time (x = viewing direction, y = image-left, z = image-up). Check them once:

1. **Frames arrive:** `ros2 topic hz /quest/passthrough` shows ~3 Hz while calibrating. The Unity log prints `[PassthroughCameraSource] K for 1280x960: fx … cy … (principal point … px from the centre)`. Expect fx ≈ fy, and the principal point within a few % of the centre.
2. **Picture:** open `/vr_tag_detector/debug_image` in `rqt_image_view` or RViz.
   - It must look like what you see: ceiling up, not mirrored.
   - If it's upside down, tick `flipVertically` on `PassthroughFrameStreamer`.
   - Tags get green outlines and axes: red x toward the printed tag's right, green y toward its top, blue z out of the paper.
3. **Pose:** look at one tag from ~0.6 m. Slowly turn your head left/right and up/down, keeping the tag in view, then press A. The tag's spread should be a few mm (≤ 10 mm).
   - A spread that grows with head **rotation** means the pose or `K` is wrong.
   - If it mostly grows when you tilt your head **up/down**, and the logged cy is far (> ~10 px) from the centre, tick `principalPointFromTop` on `PassthroughCameraSource`, press B and try again. Keep whichever setting gives the smaller spread.
4. **Calibration:** the reply's residuals should be ≤ ~15 mm. After calibrating:
   - `TagMapVisualizer`'s squares should sit on the printed tags, within ~1–2 cm, with the edges parallel.
   - In RViz, the fused tags (`/vr_tag_detector/markers`) should overlap the robot's `tag_<id>` frames.
5. **Cross-check:** touch a tag centre with the controller tip (`TagTouchRegistration`'s sphere). It should be within ~2 cm of where the passthrough calibration puts it.

### What gets streamed to ROS

`QuestStatePublisher` sends `/quest/state` at `rateHz` (30 Hz). On the ROS side, `quest_bridge` turns it into TF frames, `PoseStamped` topics in `base_link` and a `sensor_msgs/Joy` per controller. Per controller:

- **`tracked`:** from `OVRInput.GetControllerPositionTracked`, and false for everything while `RobotWorldFrame.RegistrationStale` is set (after a recenter). Before the first calibration of a session, poses are sent as tracked, and the ROS side uses its last saved calibration.
- **Pose:** the controller anchor. Its Unity forward becomes ROS x, so a controller's x axis is its pointing direction (and `quest_head`'s is the gaze direction).
- **Analog:** `index_trigger`, `hand_trigger` (grip), and the thumbstick x/y.
- **Buttons:** `button_primary` (A / X), `button_secondary` (B / Y), `thumbstick_pressed`, and `menu` (left only).

### Notes

- **Topics and services are registered by message name** (`RegisterPublisher(topic, XxxMsg.k_RosMessageName)`), never with the generic `RegisterPublisher<XxxMsg>`. In a headset build, the generated classes register their names only after the first scene's `OnEnable`, so a generic call from `OnEnable` works in the Editor but never on the headset. Do the same in your own scripts.
- **Controller buttons:** with a single controller named (`OVRInput.Controller.LTouch` / `RTouch`), X/A is `Button.One` and Y/B is `Button.Two`. `Button.Three`/`Four` only exist on the combined `Controller.Touch`.
- **Empty service requests:** ROS 2 serialises an empty message as one dummy byte, but ROS-TCP-Connector's generated empty requests serialise as nothing, and the ROS side never replies. Send `RosUtil`'s `EmptyRequest` instead (as `TagMapVisualizer` does).
- **Service timeouts:** the endpoint never replies when a service isn't running, so every service call here gives up after `serviceTimeoutS` and says so.
- **Not a Store app:** the Passthrough Camera API and these scripts are for a lab app.
- **Alternative tag source:** MRUK's QR-code trackables (`MRUKTrackable`, experimental) give the headset's own pose of a printed QR code without streaming images. The robot's tag map uses ArUco tags, not QR codes, so this would need a QR detector on the ROS side as well.
