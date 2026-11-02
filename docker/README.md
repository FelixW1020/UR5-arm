# Run the Headset Side in Docker

No Ubuntu 22.04? This container runs the headset side of the repo on macOS, Windows or Linux:
- ROS 2 Humble with the `vr_*` packages;
- the ROS-TCP-Endpoint the Quest app connects to (port 10000);
- the simulated UR5;
- a browser 3D view instead of RViz.

It has no robot driver, cameras or YOLO, so use a real Ubuntu PC for the robot.

## 1. Build the image (once, about 10 minutes)

Install and start Docker:
- **macOS:** [Docker Desktop](https://www.docker.com/products/docker-desktop/).
- **Windows:** run `wsl --install` in an admin PowerShell and restart. Then install Docker Desktop (it uses WSL 2) and Git (`winget install Git.Git`).
- **Linux:** `sudo apt install docker.io git`, then `sudo usermod -aG docker $USER` and log in again. On Ubuntu 22.04 you can skip Docker and [install natively](../README.md#install-ubuntu-2204--ros-2-humble) instead; that also runs the robot side.

For a USB connection to the Quest you also need `adb`: `brew install --cask android-platform-tools` (macOS), `winget install Google.PlatformTools` (Windows) or `sudo apt install adb` (Linux).

Then, from the repo root:

```bash
docker build -f docker/Dockerfile -t ur5-vr .
```

Rebuild after a `git pull` that adds packages or messages. Edits to Python code and config files don't need a rebuild if you mount `src/` (see below).

## 2. Check it works: the simulation

```bash
docker run --rm -it ur5-vr ros2 launch ur5_bringup vr_sim.launch.py rviz:=false controller_mode:=scripted
```

After about 4 s, the log should print `Auto-registration succeeded`. Press Ctrl+C to stop.

## 3. Test the real headset

1. **Tag map:** save it as `~/ur5_ros/handeye/tag_map.yaml` on your computer. The tag map in [the headset test](../docs/headset-test.md#3-test-it-with-the-ros-pc) works as-is.
2. **Start the container** from the repo root:
   ```bash
   docker run --rm -it --name ur5-vr -p 10000:10000 -p 8765:8765 \
     -v ~/ur5_ros:/root/.ros -v "$PWD/src:/ws/src" ur5-vr
   ```
   - `-v ~/ur5_ros:/root/.ros` keeps tag maps and registrations on your computer.
   - `-v "$PWD/src:/ws/src"` uses your local copy of the code and config, for example a changed `marker_length` in `src/ur5_bringup/config/vr_sim.yaml`.
   - **Windows PowerShell:** put it on one line (or end lines with `` ` `` instead of `\`), and write `${HOME}/ur5_ros` and `${PWD}/src`.
   - Stop it with Ctrl+C, or `docker stop ur5-vr` from another terminal.
3. **Connect the Quest.** Either way, the terminal prints `Connection from …` when it works.
   - **USB** (the app's ROS IP is `127.0.0.1`): run `adb reverse tcp:10000 tcp:10000`. Run it again after every replug.
   - **Wi-Fi:** build the app with this computer's IP: `ipconfig getifaddr en0` (macOS), `hostname -I` (Linux) or `ipconfig` (Windows, the Wi-Fi adapter's IPv4 address). Allow incoming connections if the firewall asks.
4. **Test:** follow *In the headset* in [the headset test](../docs/headset-test.md#3-test-it-with-the-ros-pc).
5. **ROS commands in a second terminal:**
   ```bash
   docker exec -it ur5-vr entrypoint.sh bash
   ros2 topic hz /quest/passthrough
   ```

## 4. See it in 3D (instead of RViz)

1. Open [app.foxglove.dev](https://app.foxglove.dev) (free account) and choose *Open connection > Foxglove WebSocket*, `ws://localhost:8765`.
2. Add a **3D** panel to see the TF frames: `base_link`, `vr_world`, the headset and the controllers.
3. Add an **Image** panel on `/vr_tag_detector/debug_image` to see what the headset camera sees.
