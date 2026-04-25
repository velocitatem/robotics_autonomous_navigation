# Husarion ROSbot Competition Workspace

This repository is initialized as a Catkin workspace for a modular ROSbot competition stack.

ROS tools are expected to run inside Docker in this repo (no host ROS installation required).

## Architecture

The system is split into independent ROS packages and nodes:

- `rosbot_navigation`: SLAM, `move_base`, `twist_mux`, and IR safety-stop helper.
- `rosbot_perception`: RGB-D puck + ArUco perception and map-frame projection with `tf2`.
- `rosbot_manipulation`: Gripper services with load-based grasp validation.
- `rosbot_mission_control`: Event-driven mission state machine.
- `rosbot_dashboard`: PyQt dashboard (annotated camera, map, and event log).
- `rosbot_competition_msgs`: Shared message/service contracts.
- `rosbot_competition_bringup`: Top-level launch orchestration.

## Docker Workflow (Recommended)

From the repository root:

```bash
docker compose build rosbot-dev
./scripts/docker_catkin_make.sh
```

Open an interactive shell with ROS sourced:

```bash
./scripts/docker_shell.sh
```

If you need GUI support for the dashboard from containerized apps:

```bash
xhost +local:docker
```

To install package dependencies manually in the container:

```bash
docker compose run --rm rosbot-dev bash -lc "rosdep install --from-paths src --ignore-src -r -y"
```

## Launch

```bash
docker compose run --rm rosbot-dev bash -lc "catkin_make && source devel/setup.bash && roslaunch rosbot_competition_bringup competition_system.launch"
```

## Main Topics and Services

- `/perception/spatial_detections` (`rosbot_competition_msgs/SpatialDetection`)
- `/perception/annotated_image` (`sensor_msgs/Image`)
- `/mission_events` (`std_msgs/String`)
- `/grasp_puck` (`rosbot_competition_msgs/GraspPuck`)
- `/release_puck` (`std_srvs/Trigger`)
- `/cmd_vel_nav`, `/cmd_vel_servo`, `/cmd_vel_safety` multiplexed to `/cmd_vel`

## Notes

- Parameters are externalized in YAML files under each package's `config/` directory.
- `move_base` output is remapped through `twist_mux` so global navigation and visual servoing do not conflict.
- The current code provides a production-ready scaffold with conservative defaults and explicit extension points.
- `docker-compose.yml` uses `network_mode: host` so ROS graph discovery works with physical robot networking.
