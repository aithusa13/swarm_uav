# Task Allocation and Autonomous Coordination for a ROS-Based UAV Swarm System

Capstone project. 5-UAV swarm (1 carrier plus 4 mini UAVs) in ROS Noetic with MAVROS over ArduPilot SITL. Each role lives in its own self-contained ROS package.

## Stack

- ROS Noetic on Ubuntu 20.04
- MAVROS + mavros_extras
- ArduCopter SITL
- Gazebo Classic
- OpenCV (DNN module and ArUco contrib)

## Packages

### human_tracking

- Searches for a person wearing a red uniform
- MobileNet SSD detection, SORT tracking, red-uniform authorization
- PID follow (distance, lateral, yaw)
- Publishes custom `PersonBBox` message

Requires

- MAVROS, cv_bridge
- OpenCV DNN, numpy, `sort` Python module
- MobileNet SSD model files (see `human_tracking/scripts/models/README.md`)
- Topics `/drone1/webcam/image_raw`, `/drone1/laser/scan`

### area_scanning

- Autonomous S-Shape or Zigzag scan
- Parameters `a` (step distance) and `k` (strip count) read from terminal
- Hover, back-off, descend sub-sequence on person detection
- RTL on scan complete or obstacle

Requires

- MAVROS, cv_bridge
- OpenCV DNN, numpy
- MobileNet SSD model files (see `area_scanning/scripts/models/README.md`)
- Topics `/drone2/webcam/image_raw`, `/drone2/laser/scan`
- No dependency on `human_tracking`, ships its own `DroneController`

### kamikaze_dive

- State machine PRECHECK, TAKEOFF, CLIMB, CRUISE, DIVE, PULLOUT, LAND
- Nose-down dive with ramped forward and vertical velocity
- Aggressive pullout at 1 m
- Mission params via rosparam (`~cruise_alt`, `~cruise_dist`, `~dive_alt`, `~dive_fwd_vel`, `~dive_dn_vel`, `~pullout_alt`)

Requires

- MAVROS with `local_position/pose`
- No camera, no lidar
- Single drone (no namespace prefix)

### object_detection

- Short 6-step area scan
- Asynchronous ArUco detection (DICT_4X4_50)
- P-controlled centering over marker
- Publishes target GPS on `/uav/detected_target_global`
- Switches to RTL after centering

Requires

- MAVROS with `global_position/global`
- OpenCV ArUco contrib, cv_bridge
- Topic `/webcam/image_raw`

### carrier

- Visits configured GPS waypoints in GUIDED mode
- Dwells at each waypoint for a set duration
- Returns to launch at the end

Requires

- MAVROS with global position fix
- Waypoint list in `carrier.py` must be edited for the actual SITL home location

## Build

```
mkdir -p ~/catkin_ws/src
cp -r human_tracking area_scanning kamikaze_dive object_detection carrier ~/catkin_ws/src/
cd ~/catkin_ws
catkin build
source devel/setup.bash
```

MAVROS and ArduPilot SITL must be installed and running before launching any node.

## Running

```
rosrun human_tracking follow_person
rosrun human_tracking person_tracker.py
```

Each package uses its own namespace (`/drone1`, `/drone2`, or none). Running two packages on the same namespace against one simulated drone will cause setpoint conflicts.

## Note

Still under active development. Known gaps include placeholder parameters, hard coded waypoints that must be adapted to a specific SITL home location, and limited error handling. Current form reflects the end of the capstone period and is being improved.
