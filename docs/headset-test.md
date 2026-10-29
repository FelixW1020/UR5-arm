# Headset Calibration Test App: Notes for the Unity Developer

**Goal:** a bare Quest 3 app that does one thing. You look at a printed tag, press A, and a square appears on the tag. This is the first time the calibration scripts run on a real headset, so the point is to find what breaks before it goes into the game. No robot needed: the ROS PC runs a simulated UR5.

The full reference is the [Unity developer guide](../unity/README.md). This page is the short version.

## 1. Build it

**You need:**

- Unity 6 with Android Build Support, and a copy of this repo.
- A Quest 3 in Developer Mode, and a USB-C cable. Developer Mode needs a verified Meta account in a developer team ([developers.meta.com](https://developers.meta.com/horizon/)); then Meta Horizon phone app → your headset → Headset settings → Developer mode.
- The ROS PC's IP address (`hostname -I` on that PC).

**Steps:**

1. Do step 1, *Project setup*, of the [Unity developer guide](../unity/README.md#1-project-setup-once).
2. In a new scene, add only these:

   | GameObject | How | Settings |
   |---|---|---|
   | Camera rig | Building Blocks **Camera Rig** and **Passthrough** (*Meta > Tools > Building Blocks*; on SDK v207+ *Window > Meta > Tools > Building Blocks*) | On the rig's OVR Manager, tick *Enable Passthrough Camera Access*. The blocks set Floor Level and passthrough for you |
   | `RobotWorldFrame` | Empty + `RobotWorldFrame` | — |
   | `Calibration` | Empty + `PassthroughCameraAccess` (from MRUK), `PassthroughCameraSource`, `PassthroughFrameStreamer`, `PassthroughTagRegistration` | `PassthroughCameraAccess`: keep the defaults (Left, 1280×960), **component unticked**. `PassthroughFrameStreamer`: **component unticked** |
   | `TagMap` | Empty + `TagMapVisualizer` | Tag Size = the printed black square (0.10). On `PassthroughTagRegistration`'s *On Registered*, add `TagMap` → `TagMapVisualizer.Refresh` |
   | Status label (recommended) | Right-click `CenterEyeAnchor` > *3D Object > Legacy > TextMesh* | Position z 0.8, Scale 0.01, Anchor *Middle center*. Drag it into `PassthroughTagRegistration`'s *Status*. Some messages ("Hold still", camera permission) only show here |

3. Save the scene. In *File > Build Profiles* (Android), click *Add Open Scenes* under *Scene List* and untick every other scene. Plug in the Quest, accept *Allow USB debugging* in the headset, pick it as *Run Device*, and click *Build And Run*.

## 2. Be mindful of

**Setup traps (these fail silently)**

- **The `ROS2` define.** Choose ROS2 in *Robotics > ROS Settings* while Android is the active platform, then check that *Player Settings > Android > Scripting Define Symbols* contains `ROS2`. Without it the app connects, but the ROS PC can't read anything it sends.
- **The ROS IP is built into the app.** If the PC's IP changes (a new network, a hotspot), rebuild. The Quest and the PC need a network that lets devices talk to each other: campus Wi-Fi often doesn't, a lab router or phone hotspot does.
- **Generate messages only from `src/vr_interfaces`.** `std_msgs`, `geometry_msgs`, `sensor_msgs` and `builtin_interfaces` are already in ROS-TCP-Connector; duplicates won't compile.
- **Versions:** MRUK v81 or newer. MRUK v203+ needs Unity 6 (6000.0.66f2+); Unity 2022.3 stops at MRUK v201.

**Passthrough camera**

- The two unticked components stay unticked: the scripts switch the camera on only while calibrating.
- Only one `PassthroughCameraAccess` per camera. If the game also uses the camera, share it ([how](../unity/README.md#2-add-these-objects-to-your-scene)).
- `PassthroughCameraSource` asks for the camera permission. Don't also tick OVR Manager's start-up permission request: ask from one place only.
- If a Meta SDK update breaks the camera code, the fix belongs in `PassthroughCameraSource.cs` only.
- The headset camera API is for lab apps, not the Store.

**World and tracking**

- Keep Tracking Origin Type on *Floor Level*. Never move `OVRCameraRig` or `RobotWorldFrame` from code (no teleport or smooth locomotion): it slides the virtual world off the real robot.
- Recentering (holding the Meta button) breaks the calibration. `RobotWorldFrame` notices, sets `IsRegistered` to false and fires `onRegistrationLost`. Calibrate again.
- Calibrate from the headset, every session. A calibration started from the ROS PC doesn't fire `onRegistered`.

**Your own code that talks to ROS**

- Register topics and services **by message name**: `RegisterPublisher(topic, XxxMsg.k_RosMessageName)`, not `RegisterPublisher<XxxMsg>(topic)`. The generic form works in the Editor but not on the headset.
- For an empty service request, send `RosUtil`'s `EmptyRequest`. A generated empty request never gets a reply.
- Subscriptions outlive destroyed objects: start callbacks with `if (this == null) return;`.
- ROS and Unity axes differ (ROS: x forward, y left, z up). Convert with ROS-TCP-Connector's `ROSGeometry` (`new Vector3<FLU>(x, y, z).toUnity`, `.To<FLU>()`) instead of swapping axes by hand.
- `PassthroughTagRegistration` uses the right controller's A and B. For your own UI, untick `useControllerButtons`, call `Register()` and `ResetSightings()`, and show `StatusText` (`IsBusy` is true while waiting). Enable only one registration script at a time.

## 3. Test it with the ROS PC

**On the ROS PC** (Ubuntu 22.04 + ROS 2 Humble with this repo built, see [Install](../README.md#install-ubuntu-2204--ros-2-humble)):

1. Print tag 0 and tape it flat on a table:
   ```bash
   ros2 run aruco_tag_mapper generate_tags --ids 0
   ```
   This writes `aruco_tags/tag_0.png`. Print it so the whole image is **12.5 cm wide** (it's made for 300 dpi; "100%" in a print dialog usually isn't). The black square is then 10 cm.
2. Save this as `~/headset_test_tag_map.yaml`:
   ```yaml
   frame_id: base_link
   dictionary: DICT_4X4_50
   marker_length: 0.10          # the black square, in metres
   tags:
     0:
       position: {x: 0.50, y: 0.0, z: 0.0}
       orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
   ```
   There's no real robot, so these numbers only place a pretend robot 50 cm to the left of the tag (as you read it). If your square isn't 10 cm, put your measurement here, under `vr_tag_detector` in `src/ur5_bringup/config/vr_sim.yaml`, and in `TagMapVisualizer`'s Tag Size.
3. Start everything and leave it running:
   ```bash
   ros2 launch ur5_bringup vr_sim.launch.py quest:=real tag_map_file:=$HOME/headset_test_tag_map.yaml
   ```

**In the headset:**

1. Open the app and allow the camera permission. The ROS terminal prints `Connection from <Quest IP>`.
2. Look at the tag from about 0.5 m and hold still until the ROS terminal shows `In view: tag 0 at … m`. The app only takes pictures while your head is still.
3. Turn your head a little to the left and hold still for ~2 s. Do the same to the right, up and down.
4. Press **A**. The ROS terminal prints a line like `tag 0: n=24, spread 4.2 mm`. **B** starts over. If nothing prints, the tag hasn't been seen enough yet (the status label says so): keep looking and press A again.

**It works if:**

- `n` is 15 or more and the **spread** is 10 mm or less, and
- the square sits on the printed tag, and stays on it as you walk around.

**If it doesn't:**

| Problem | Fix |
|---|---|
| No `Connection from …` line | Check the IP in *Robotics > ROS Settings* and rebuild after changing it. Both devices must be on the same network |
| Spread over 10 mm, or the square is off the tag | Press **B** (a successful A turns the camera off). On the ROS PC, run `ros2 run rqt_image_view rqt_image_view` and pick `/vr_tag_detector/debug_image`. The picture must be upright and not mirrored. If it's upside down, tick `flipVertically` on `PassthroughFrameStreamer` and rebuild |
| Spread is bigger after looking only up and down than only left and right (press B between tries) | Tick `principalPointFromTop` on `PassthroughCameraSource`, rebuild, and keep it only if the spread goes down |
| Anything else | The [first-build checklist](../unity/README.md#first-build-checklist) |

## 4. Send back after the first run

The scripts make three guesses about the headset's camera that no headset has checked yet. Please report:

- the `[PassthroughCameraSource] K for 1280x960: …` line from the device log (Meta Quest Developer Hub's log viewer, or `adb logcat -s Unity`);
- the `n` and **spread** from the reply after pressing A;
- whether `/vr_tag_detector/debug_image` was upright, and whether you needed `flipVertically` or `principalPointFromTop`;
- whether the square sat on the printed tag.
