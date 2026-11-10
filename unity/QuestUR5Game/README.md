# Quest–UR5 Collaboration Demo

Unity gameplay prototype by Zheng He for an AR collaboration project with Felix Wang and the Pajic lab's UR5 robot.

The demo explores object pickup, placement on designated pads, simulated robot assistance, task feedback, and CSV-based trial recording. Available behavior depends on the scene. Robot animations in a Unity scene do not establish that the physical UR5 is connected.

**Start here:** clone this whole repository, install the Unity Editor version specified by this project, then open the agreed demo scene. You do not need to download each scene separately.

> **Handoff item for Zheng:** replace `[MAIN_SCENE_PATH]` below with the exact path of the scene Felix should run first, such as the path shown for that scene inside `Assets`. This path has not yet been verified. Also tell Felix which Meta XR Simulator version you used if he will test without a headset.

## 1. Which download do I need?

| Goal | Download / use | What it provides |
| --- | --- | --- |
| Inspect, run, or modify the Unity project | **Clone this entire repository**, preferably with [GitHub Desktop](https://desktop.github.com/) | Scenes, scripts, assets, package configuration, and project settings |
| Install a ready-made demo on Quest 3 | A **separately supplied Android `.apk`** from Zheng, if available | An installable app; not editable Unity source |
| Work on robot tracking, registration, or ROS integration | [Felix's UR5-arm repository](https://github.com/FelixW1020/UR5-arm) | Robot-side tools and Unity registration components; not a replacement for this gameplay project |

GitHub's **Code → Download ZIP** downloads source code, not an APK. Cloning is preferred for collaboration and for repositories using Git LFS. If using ZIP, extract it before opening it and ensure any LFS-managed assets were included.

For a private repository, accept Zheng's collaborator invitation and sign in to the invited GitHub account.

## 2. Software to install

| Software | Required when | Version / installation rule |
| --- | --- | --- |
| [Unity Hub](https://unity.com/download) | Opening or building the source project | Use it to install and manage the Unity Editor |
| Unity Editor | Opening or building the source project | **Use the exact `m_EditorVersion` in [ProjectSettings/ProjectVersion.txt](ProjectSettings/ProjectVersion.txt)** |
| Android Build Support, Android SDK & NDK Tools, OpenJDK | Building an APK for Quest 3 | Install these modules for that exact Editor version through Unity Hub |
| Project packages, including XR dependencies | Opening the source project | Restore the versions declared in [Packages/manifest.json](Packages/manifest.json) and [Packages/packages-lock.json](Packages/packages-lock.json), where present |
| Meta XR Simulator | Testing in the Editor without a physical headset | Match Zheng's tested simulator setup; see the simulator notes below |
| GitHub Desktop | Recommended for downloading and collaborating | Handles cloning, commits, and synchronization |
| ROS / Docker | A later, separate robot-integration test | Follow Felix's repository for that test |

**Do not choose an Editor version just because it is the newest release, and do not copy the Unity version from Felix's separate calibration test app.** This project's version file is the reference.

If that Editor is not listed in Unity Hub, use the [Unity download archive](https://unity.com/releases/editor/archive).

To add Android modules to an existing Editor installation, open **Unity Hub → Installs**, use that Editor's menu, and choose **Add modules**. Select Android Build Support and its SDK/NDK and OpenJDK components.

### Package restoration

Open the project and let Unity Package Manager finish resolving dependencies before changing anything. The repository's package configuration should determine package versions.

- Do not install every Meta package manually or upgrade packages as a first troubleshooting step.
- Assets imported directly into `Assets` must be included in the repository with their `.meta` files.
- A package dependency using a `file:` path outside the project needs separate handling. If it points to a directory on Zheng's computer, ask Zheng to include or relocate that dependency.
- Git-based package dependencies may require Git to be installed and available to Unity.
- External programs, such as a simulator runtime, are not automatically transferred just because Unity package files are present.

## 3. Open the project

1. Clone this repository to a local directory.
2. Confirm that the Unity project root contains `Assets`, `Packages`, and `ProjectSettings`.
3. Read `ProjectSettings/ProjectVersion.txt` and install the matching Editor.
4. In **Unity Hub → Projects → Add → Add project from disk**, select that project root.
5. Open it and wait for package restoration, asset import, and script compilation.
6. Open the Console. Resolve compilation errors before testing interactions.
7. Open the agreed entry scene:

   **Main demo scene: `[MAIN_SCENE_PATH]`**

The project contains development iterations. Ask Zheng which scene is the current handoff scene; the first scene found in the project may be an earlier experiment.

A fresh clone does not include `Library`. Unity regenerates that cache, so the first import can take longer than subsequent launches.

## 4. Choose a test route

### A. Meta XR Simulator — no headset

1. Confirm which simulator release and activation method Zheng used successfully.
2. Install the corresponding runtime for your computer.
3. Open the main scene, activate that simulator using the controls appropriate to the installed release, and enter Play mode.
4. Inspect the simulator's input bindings for controller movement, grip, and selection.
5. Test pickup and release using those bindings.

Keyboard shortcuts can vary between simulator releases and local configurations. Do not assume a particular key, such as `U`, is universally mapped to grip.

**Version note:** Meta's current documentation describes a standalone simulator, while archived documentation describes older workflows. Use the documentation matching the tested setup. Do not migrate SDKs or remove existing packages just to reproduce the demo.

- [Current Meta XR Simulator setup](https://developers.meta.com/vr/documentation/unity/xrsim-getting-started/)
- [Archived simulator setup](https://developers.meta.com/vr/documentation/unity/xrsim-getting-started-1.0/)

### B. Physical Quest 3 — Android build

1. Install the Android modules listed above.
2. Use a Quest 3 with Developer Mode enabled.
3. Connect it with a USB data cable and accept the USB debugging authorization prompt in the headset.
4. Open **File → Build Profiles**, select or create the Android profile, and switch to it if necessary.
5. Verify that the intended demo scene is included and that the profile uses the expected startup scene.
6. Select the connected device and use **Build and Run**.
7. Put on the headset, grant permissions requested by the app as appropriate, and test the interaction flow.

Keep the committed XR/player configuration for the first reproduction. A successful Windows simulator run does not by itself verify the Android build.

If Zheng has already provided a working APK, it can be installed separately without building the Unity source.

## 5. First functional check

Use the behavior enabled in the selected scene:

1. Confirm that the scene, pads, and task/status display are visible.
2. Pick up and release the interactive cube.
3. Place it in the designated target area and check the feedback.
4. Check any configured destination selection or simulated robot-assistance sequence.
5. Verify that timing, mistakes, and task-state feedback update as expected.
6. Finish the trial/session and verify that the CSV export succeeds.

Report the scene path, test route, and first relevant Console error if something fails. A screenshot of the Hierarchy and the affected object's Inspector is useful for missing references.

### CSV output

Task data is written by the logging script rather than by GitHub. Use the output path printed by the script in the Unity Console or device logs.

The shared `QuestPlacementGoal.cs` implementation uses `Application.persistentDataPath`, a `PlacementResults` subfolder, and session-specific `placement_*.csv` filenames. Confirm this against the checked-out script if logging has changed.

The actual base directory depends on the platform and project identity. CSV files generated on Quest are on the headset; they do not automatically appear in the PC project folder or GitHub repository. Use the CSV header and logging code as the reference for each metric's meaning.

## 6. Relationship to the UR5 system

This project provides gameplay, interaction, feedback, and trial recording. Felix's repository provides the ROS and spatial-registration work to integrate with it:

- [System overview](https://github.com/FelixW1020/UR5-arm/blob/main/README.md)
- [Unity integration guide](https://github.com/FelixW1020/UR5-arm/blob/main/unity/README.md)
- [Headset test results](https://github.com/FelixW1020/UR5-arm/blob/main/docs/headset-test-results/README.md)

Before connecting a scene to the real robot, agree on:

- The tested registration configuration and coordinate frames.
- Which real object and source/target locations the task uses.
- The task-request interface and accepted/running/completed/failed feedback.
- How registration loss, disconnection, and cancellation affect the game.
- Whether grip-based robot teleoperation is enabled; it can conflict with gameplay grabbing.

Physical robot execution is a separate integration milestone. Neither a Unity animation nor the publication of a target pose proves that a physical pick-and-place task has completed.

## 7. Collaboration

- Keep asset files and their matching `.meta` files together.
- Commit `Assets`, `Packages`, and `ProjectSettings`; leave generated caches and build outputs out of source control.
- Coordinate changes to the same scene or prefab.
- Make package upgrades and robot-integration changes on a separate branch.
- Keep the working gameplay scene available while testing integration.

**Handoff is verified when Felix can open the correct scene with the matching Editor, run the intended interaction route, and locate a successfully exported CSV.**

## Official setup references

- [Manage Unity Hub projects](https://docs.unity.com/en-us/hub/project-manage)
- [Install Unity Editor versions](https://docs.unity.com/en-us/hub/add-editor)
- [Android development dependencies](https://docs.unity3d.com/6000.0/Documentation/Manual/android-install-dependencies.html)

