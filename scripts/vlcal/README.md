# HOUND ↔ direct_visual_lidar_calibration (vlcal)

Calibrate each RealSense (`camera_front` / `camera_left` / `camera_right`) to the
Livox Mid-360S **independently**. Cameras do not need overlapping views.

**Jetson note:** the Hub image `koide3/direct_visual_lidar_calibration:jazzy` is
**amd64-only**. On this arm64 box, build and run **natively inside `mushr_jazzy`**.

Upstream docs: https://koide3.github.io/direct_visual_lidar_calibration/

## What survives Docker exit (no commit)

| Path | Survives? | Notes |
|------|-----------|--------|
| `colcon_ws/src/direct_visual_lidar_calibration/` | yes (mount) | source + thirdparty + HOUND patches |
| `colcon_ws/src/hound_core/scripts/vlcal/` | yes | runners / SSoT helpers |
| `colcon_ws/calib/vlcal/` | yes | bags + preprocessed + `calib.json` |
| `colcon_ws/install/direct_visual_lidar_calibration/` | yes (mount) | binaries, but need `/usr/local` libs at runtime |
| `/usr/local` GTSAM + Iridescence | **no** | must reinstall after fresh container |
| apt `libceres-dev` etc. | **no** (unless baked into image) | reinstall via script |

## One-shot reinstall after fresh `mushr_jazzy`

```bash
mushr_jazzy
# inside container:
source /opt/ros/jazzy/setup.bash
bash /root/colcon_ws/src/hound_core/scripts/vlcal/install_vlcal_deps.sh
```

That script:

1. `apt` installs Ceres, PCL, OpenCV, Boost, GLFW, ROS cv-bridge, …
2. Inits vlcal git submodules (`json`, `nanoflann`, `Sophus`)
3. Clones **Bonxai** into `thirdparty/Bonxai` and symlinks `include → bonxai_core/include`
4. Builds **GTSAM 4.2a9** → `/usr/local` (`-DGTSAM_WITH_TBB=OFF`)
5. Builds **Iridescence** → `/usr/local`
6. `colcon build --packages-select direct_visual_lidar_calibration`

GTSAM + Iridescence take a while on Orin (tens of minutes). Source trees under
`/tmp/vlcal_deps` can be reused within the same container session.

Optional: commit the container after a successful install so you don’t rebuild
`/usr/local` every time:

```bash
# on host
docker commit mushr_jazzy mushr_jazzy:vlcal
```

## Layout

```text
/root/colcon_ws/calib/vlcal/          # same disk as /home/hound/colcon_ws/calib/vlcal
  rosbag2_.../                        # raw bag(s)
  camera_front/bags/ → ../../rosbag2_...
  camera_front/preprocessed/
  camera_left/...
  camera_right/...
```

## Phase 1 — Record (lidar on)

Inside `mushr_jazzy`, with Livox + cameras up, robot still:

```bash
ros2 bag record -a -o /root/colcon_ws/calib/vlcal/rosbag2_$(date +%Y_%m_%d-%H_%M_%S)
```

Link into camera folders (one bag can serve all three) — or use the runner:

```bash
cd /root/colcon_ws/src/hound_core/scripts/vlcal
./run_vlcal.sh use_bag rosbag2_YYYY_MM_DD-HH_MM_SS
# replaces prior bag links under camera_*/bags/
```

Manual equivalent:

```bash
BAG=/root/colcon_ws/calib/vlcal/rosbag2_YYYY_MM_DD-...
for c in camera_front camera_left camera_right; do
  mkdir -p /root/colcon_ws/calib/vlcal/$c/bags
  ln -sfn "../../$(basename "$BAG")" /root/colcon_ws/calib/vlcal/$c/bags/$(basename "$BAG")
done
```

Then stop the lidar cleanly and cut power.

## Phase 2 — Offline calibrate

Needs `DISPLAY` / X11 for GUI steps (or use `initial_guess_ssot`):

```bash
source /opt/ros/jazzy/setup.bash
source /root/colcon_ws/install/setup.bash
cd /root/colcon_ws/src/hound_core/scripts/vlcal

BAG=rosbag2_2026_08_11-01_48_52   # your new bag name
./run_vlcal.sh use_bag "$BAG"

for cam in camera_front camera_left camera_right; do
  ./run_vlcal.sh preprocess "$cam"          # or: preprocess "$cam" "$BAG"
  # GUI: ≥3 right-click pairs → Add → Estimate → Save
  ./run_vlcal.sh initial_guess_manual "$cam"
  # Or seed from SSoT mounts (no GUI):
  # ./run_vlcal.sh initial_guess_ssot "$cam"
  ./run_vlcal.sh calibrate "$cam"
done

python3 ./apply_vlcal_to_ssot.py --no-tf          # dry-run
python3 ./apply_vlcal_to_ssot.py --no-tf --write  # patch SSoT.yaml
```

`run_vlcal.sh` prepends `/usr/local/lib` to `LD_LIBRARY_PATH` and remaps
RealSense `inverse_plumb_bob` → `plumb_bob` after preprocess.

`run_vlcal_docker.sh` is legacy (amd64 Hub image) — prefer `run_vlcal.sh` on Jetson.

## SSoT mapping

| Result | SSoT field |
|--------|------------|
| `base_link` → `camera_front_link` | `hal_monitor.camera.pos` / `rot` (xyzw) and `stereo_composite.cameras.camera_front.xyz/rpy` |
| `camera_front_link` → `camera_left_link` | `stereo_composite.cameras.camera_left.xyz/rpy` |
| `camera_front_link` → `camera_right_link` | `stereo_composite.cameras.camera_right.xyz/rpy` |

## Notes

- Livox: static bags, intensity channel `intensity`, no `-d` dynamic integrator.
- Default preprocess `--min_distance 0.2` (vlcal’s 1.0 m default drops near-field Mid-360 points).
- Manual initial guess, or `initial_guess_ssot` from existing mounts.
- Mid-360S has **core_temp** only (no environment temperature reading).
