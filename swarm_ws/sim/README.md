# Simulation assets

The ROS 2 packages in `src/` expect a Gazebo world called `maze_survivors` with
drones publishing on `/drone1/...` and `/drone2/...`. None of that was in the
repository, so it is rebuilt here.

```
sim/
  tools/gen_drone_models.py   generates the two sensor-equipped drone models
  tools/gen_world.py          generates worlds/maze_survivors.sdf
  worlds/maze_survivors.sdf   the maze (generated, committed)
  models/                     the drone models (generated, git-ignored)
  config/drone1.parm          per-drone MAVLink / DDS parameters
  config/drone2.parm
  run_swarm.sh                brings the whole thing up
  logs/<timestamp>/           one log per component, per run
```

## Requirements

* ROS 2 Jazzy, Gazebo Harmonic (`gz sim` 8.x)
* ArduPilot SITL built (`~/ardupilot/build/sitl/bin/arducopter`) with AP_DDS
* [ardupilot_gazebo](https://github.com/ArduPilot/ardupilot_gazebo) built
* `MicroXRCEAgent` on `PATH`
* `ardupilot_msgs` on the prefix path (e.g. `~/ardu_ws/install`)
* `pip install --user --break-system-packages ai_edge_litert` for the detector.
  Do not let that pull NumPy 2 in: the distro OpenCV and `cv_bridge` are built
  against NumPy 1.x and fail to import under 2.x.

Override the default locations with `ARDUPILOT_HOME`, `ARDUPILOT_GAZEBO_HOME`,
`ARDU_WS` and `ROS_DISTRO_SETUP`.

## Running

```bash
colcon build --symlink-install      # from swarm_ws/
./sim/run_swarm.sh                  # 2 drones, Gazebo GUI, full ROS stack
./sim/run_swarm.sh --drones 1       # single drone
./sim/run_swarm.sh --headless       # no GUI
./sim/run_swarm.sh --no-stack       # Gazebo + autopilots only
./sim/run_swarm.sh --record run.mp4 # also film the mission
```

The script generates any missing assets, waits for Gazebo to advertise the lidar
before starting the autopilots, and tears every process group down on Ctrl-C.

Watch it with `ROS_DOMAIN_ID=1 rviz2` (`/map`, `/scan`, `/person_map_pins`,
`/exploration/planned_path`) or attach a GCS to `udpin:127.0.0.1:14551`.

## Recording a mission

`--record` films drone 1's mission to an mp4 in the run's log directory. It
works headless and reads only ROS topics, so it records the simulation rather
than whatever is on your screen - useful, since it means no X server, no
`ffmpeg` and no screen recorder are needed (OpenCV writes the file).

Three panels: the overhead camera that lives in the world, the detector's
annotated view from drone 1, and the SLAM map with the planned path, the flight
trail and the survivor pins - plus a status bar with the armed state, flight
mode, survivors pinned and elapsed time. Recording stops when the vehicle
disarms after landing, or after `--duration` seconds.

Standalone, against a stack that is already running:

```bash
ROS_DOMAIN_ID=1 python3 sim/tools/record_mission.py --out mission.mp4
```

The overhead camera is a static model in the world publishing on
`/observer/image`; `run_swarm.sh` bridges it only when `--record` is given.

## The drone models

`gen_drone_models.py` reads ardupilot_gazebo's `iris_with_ardupilot` model and
splices a sensor payload onto it, rewriting the model name and the ArduPilot FDM
port. Nothing from ardupilot_gazebo (LGPL-3.0) is committed here; the models are
generated locally and git-ignored.

Sensor geometry is not arbitrary - it matches what the ROS nodes already assume:

| Sensor | Pose w.r.t. `base_link` | Spec | Assumed by |
| --- | --- | --- | --- |
| 2D lidar | `0 0 0.05` | 360 samples, 0.2-12 m, 20 Hz | `drone.urdf`, rf2o, slam_toolbox |
| RGB camera | `0.10 0 0.03`, pitch 0.4109 rad | 640x480, hfov 1.03 rad, 15 Hz | `person_detector.py` |
| Depth camera | same pose and FOV | 640x480 float metres | `person_detector.py` |

The depth camera has to share the RGB camera's pose, FOV and resolution: the
detector reads the depth image at the RGB bounding box's pixel coordinates.

## The world

`gen_world.py` carves a seeded perfect maze, knocks a few walls through for
loops, then **proves** the result before writing it: it flood-fills from drone
1's start square and fails if any free square is unreachable or if the two start
squares are not connected. Survivors go to the dead ends furthest from both
launch points, spread as far apart as the maze allows - the detector merges two
mannequins in one corridor into a single pin.

Defaults: 11x11 squares, 26.4 m across, 2.4 m corridors, 4 survivors
(Rescue Randy, from Gazebo Fuel), seed 7.

```bash
python3 sim/tools/gen_world.py --seed 12 --survivors 5
```

`--survivors` must match `target_survivor_count` in `beta_stack.launch.py`.

## Ports

Everything is derived from the SITL instance index, so a third drone is
instance 2 and the next slot in each column.

| | drone 1 | drone 2 |
| --- | --- | --- |
| SITL instance | 0 | 1 |
| `ROS_DOMAIN_ID` | 1 | 2 |
| Gazebo FDM (ArduPilotPlugin) | 9002 | 9012 |
| MAVLink, vision estimate | 14550 | 14560 |
| MAVLink, spare for a GCS | 14551 | 14561 |
| Micro-XRCE-DDS agent | 2019 | 2020 |

## Notes on the rest of the repo

* `src/rf2o_laser_odometry` is committed as an empty directory. Clone
  `https://github.com/MAPIRlab/rf2o_laser_odometry` (`ros2` branch) into it.
* The committed `build/` and `install/` directories hold CMake caches pointing
  at another machine; delete them before the first build.
* `slam_mapping.launch.py` is the older single-drone variant and bridges topic
  names this world does not publish. `beta_stack.launch.py` is the one to use.
