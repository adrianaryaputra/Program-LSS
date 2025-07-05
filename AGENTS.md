# Project LSS Agent Instructions

## 1. Project Overview

This project, named "Program LSS", is a ROS (Robot Operating System) catkin workspace designed for controlling an autonomous surface vehicle (ASV), likely a boat. It encompasses functionalities for autonomous navigation, vehicle control, localization, and a sophisticated Ground Control Station (GCS) featuring a custom RViz plugin for enhanced visualization and user interaction.

## 2. Key Technologies

- **Framework**: ROS (Robot Operating System - Melodic/Noetic inferred from typical package formats)
- **Build System**: Catkin
- **Primary Languages**:
    - Python: Used for high-level autonomy logic, mission scripting, and vehicle control nodes.
    - C++: Used for performance-critical tasks, such as the RViz plugin and map image processing.
- **Visualization**: RViz, extended with a custom plugin (`rviz_plugin`).
- **Inter-Process Communication**: ROS Topics, Services, and Actions. Custom services are defined in `lss_srvs`.

## 3. Project Structure (ROS Packages in `src/`)

The workspace is organized into several ROS packages:

-   **`lss_autonomy`** (Python):
    -   **Purpose**: Manages high-level autonomous behaviors, including mission execution (e.g., waypoint following), path planning, and decision-making.
    -   **Key Files**: `src/lss_autonomy/src/mission_control.py`, `src/lss_autonomy/src/missions/`, `src/lss_autonomy/src/utils/`.
    -   **Dependencies**: `geometry_msgs`, `nav_msgs`, `rospy`, `sensor_msgs`, `std_msgs`, `visualization_msgs`.

-   **`lss_controller`** (Python):
    -   **Purpose**: Handles low-level vehicle control, translating desired movements into actuator commands. Likely implements a differential drive controller.
    -   **Key Files**: `src/lss_controller/src/differential_control.py`.
    -   **Dependencies**: `geometry_msgs`, `rospy`, `std_msgs`.

-   **`lss_localization`** (Python):
    -   **Purpose**: Responsible for estimating the vehicle's state (position, orientation).
    -   **Key Files**: `src/lss_localization/src/boat_tf.py` (suggests TF frame management for the boat).
    -   **Dependencies**: `rospy`.

-   **`lss_srvs`** (Service Definitions):
    -   **Purpose**: Defines custom ROS service types used for communication between various nodes in the system.
    -   **Key Files**: `srv/StringService.srv` (example).
    -   **Dependencies**: `message_generation`, `message_runtime`, `geometry_msgs`, `std_msgs`.

-   **`lss_gcs/`** (Directory for GCS-related packages):
    -   **`map_image`** (C++):
        -   **Purpose**: Provides and processes map imagery for display in the GCS.
        -   **Key Files**: `src/map_node.cpp`.
        -   **Dependencies**: `cv_bridge`, `roslib`, `roscpp`, `sensor_msgs`.
    -   **`nala_gcs_bringup`** (Launch Files):
        -   **Purpose**: Contains ROS launch files to start various parts of the system, particularly the GCS and potentially simulation environments.
        -   **Key Files**: `launch/all.launch` (primary system launch file), `launch/map.launch`.
        -   **Dependencies**: `catkin`.
    -   **`rviz_plugin`** (C++):
        -   **Purpose**: A custom RViz plugin providing an interactive interface for monitoring and controlling the ASV. Features likely include manual control inputs, telemetry display, waypoint management, and mission parameter settings.
        -   **Key Files**: Extensive C++ source and header files (`src/`, `include/`), UI definition files (`ui/`), custom message (`msg/`) and service (`srv/`) definitions specific to the plugin, and `plugin_description.xml`.
        -   **Dependencies**: `rviz`, `roscpp`, `nav_msgs`, `sensor_msgs`, `diagnostic_msgs`, `geometry_msgs`, `cv_bridge`, `message_generation`, `message_runtime`, `rosbag`.

## 4. Top-Level Mission Scripts (in repository root)

-   `mission control_ILOS.py`
-   `mission control_PLOS.py`
    -   **Purpose**: These Python scripts likely serve as high-level entry points for initiating specific types of missions (ILOS and PLOS - specific meanings would require further code inspection). They probably interface with the `lss_autonomy` package.

## 5. Development Workflow

1.  **Build the Workspace**:
    -   Navigate to the root of the catkin workspace (e.g., `~/catkin_ws/`).
    -   Run: `catkin_make`

2.  **Source the Environment**:
    -   After a successful build, in every new terminal, source the setup file:
        `source devel/setup.bash` (or `.zsh` depending on your shell)

3.  **Running the System**:
    -   **Full System**: `roslaunch nala_gcs_bringup all.launch`
    -   **Individual Nodes**: `rosrun <package_name> <executable_name>`
    -   **Mission Scripts**: `python mission control_ILOS.py` (likely after `roscore` and other necessary nodes are running).

## 6. Code Conventions and Best Practices

-   **ROS Standards**: Adhere to standard ROS naming conventions for nodes, topics, services, and parameters.
-   **Python**: Follow PEP 8 style guidelines.
-   **C++**: Maintain consistency with the existing C++ style within the package being modified.
-   **Commit Messages**: Write clear, concise, and descriptive commit messages.
-   **AGENTS.md**: This file should be kept up-to-date with any significant changes to the project structure, build process, or core functionalities to aid future agent interactions.

## 7. Key Files for Initial Understanding

-   `AGENTS.md` (this file)
-   `src/lss_gcs/nala_gcs_bringup/launch/all.launch` (to see what nodes are started)
-   `src/lss_autonomy/src/mission_control.py` (core autonomy logic)
-   `src/lss_gcs/rviz_plugin/` (directory for the custom Rviz interface)
-   `mission control_ILOS.py` and `mission control_PLOS.py` (top-level mission execution)
-   `package.xml` files within each package for dependencies.

## 8. Ship Data Logger (`ship_data_logger.py`)

A Python script named `ship_data_logger.py` is available at the root of this repository. Its purpose is to subscribe to various ROS topics, collect key USV (Unmanned Surface Vehicle) parameters, and log them to a CSV file.

### 8.1. Functionality

-   Initializes a ROS node.
-   Subscribes to multiple ROS topics to gather data.
-   Transforms and calculates some parameters (e.g., NED velocities from body velocities, global ground speed from NED velocities).
-   Writes the collected data row-by-row to a CSV file.
-   Includes error handling for missing optional dependencies like `tf.transformations` and `numpy`.

### 8.2. How to Run

1.  Ensure `roscore` is running.
2.  Ensure the necessary data provider nodes are running (e.g., MAVROS publishing IMU, GPS, and local velocity data).
3.  From the root of the repository (or the `/catkin_ws/src/usv_project_source/` directory if inside the Docker container):
    ```bash
    python3 ship_data_logger.py
    ```

### 8.3. Configuration Parameters (ROS Parameters)

The script can be configured using ROS parameters:

-   `~output_file` (string, default: `ship_data_logged.csv`): The name of the CSV file to be created.
-   `~log_rate_hz` (float, default: `1.0`): The rate (in Hz) at which data rows will be written to the CSV file.

Example usage with parameters:
```bash
rosrun your_package_name ship_data_logger.py _output_file:=my_ship_data.csv _log_rate_hz:=10.0
# (Assuming ship_data_logger.py is made part of a ROS package to use rosrun)
# Or, if running directly and parameters are set on the parameter server beforehand:
# rosparam set /ship_data_logger/output_file "my_ship_data.csv"
# python3 ship_data_logger.py
```

### 8.4. Logged Parameters

The following parameters are logged to the CSV file:

| Parameter Name                 | Unit(s)        | Source / Description                                                                 |
|--------------------------------|----------------|--------------------------------------------------------------------------------------|
| `timestamp`                    | ISO 8601       | System timestamp when the data row is prepared.                                      |
| `rudder_angle`                 | (radians?)     | Commanded rudder angle. Source: `/autonomy/pathfollowing` (Twist msg, `angular.z`).    |
| `thruster_%`                   | (0.0-1.0?)     | Commanded thruster percentage/effort. Source: `/autonomy/pathfollowing` (Twist msg, `linear.x`). |
| `pos_NED_x`                    | meters         | Position North (Local Frame). Source: `/asv/odom` (Odometry msg).                      |
| `pos_NED_y`                    | meters         | Position East (Local Frame). Source: `/asv/odom` (Odometry msg).                       |
| `pos_GLB_lat`                  | degrees        | Latitude (Global Frame WGS84). Source: `/mavros/global_position/global` (NavSatFix msg). |
| `pos_GLB_long`                 | degrees        | Longitude (Global Frame WGS84). Source: `/mavros/global_position/global` (NavSatFix msg).|
| `pos_GLB_alt`                  | meters         | Altitude (Global Frame WGS84). Source: `/mavros/global_position/global` (NavSatFix msg). |
| `acc_BDY_x`                    | m/s^2          | Body Frame X-axis acceleration. Source: `/mavros/imu/data` (Imu msg, `linear_acceleration.x`). |
| `acc_BDY_y`                    | m/s^2          | Body Frame Y-axis acceleration. Source: `/mavros/imu/data` (Imu msg, `linear_acceleration.y`). |
| `acc_BDY_z`                    | m/s^2          | Body Frame Z-axis acceleration. Source: `/mavros/imu/data` (Imu msg, `linear_acceleration.z`). |
| `vel_BDY_x`                    | m/s            | Body Frame X-axis velocity. Source: `/mavros/local_position/velocity_body` (TwistStamped msg). |
| `vel_BDY_y`                    | m/s            | Body Frame Y-axis velocity. Source: `/mavros/local_position/velocity_body` (TwistStamped msg). |
| `vel_BDY_z`                    | m/s            | Body Frame Z-axis velocity. Source: `/mavros/local_position/velocity_body` (TwistStamped msg). |
| `vel_NED_x`                    | m/s            | Calculated North velocity (Local Frame). Derived from `vel_BDY_` and current attitude.   |
| `vel_NED_y`                    | m/s            | Calculated East velocity (Local Frame). Derived from `vel_BDY_` and current attitude.    |
| `vel_NED_z`                    | m/s            | Calculated Down velocity (Local Frame). Derived from `vel_BDY_` and current attitude.    |
| `vel_GLB_ground_speed`         | m/s            | Calculated horizontal ground speed. Derived from `vel_NED_x` and `vel_NED_y`.        |
| `vel_GLB_course_over_ground`   | radians        | Calculated course over ground. Derived from `vel_NED_x` and `vel_NED_y`. (0 is North, positive East). |
| `vel_GLB_vertical_speed`       | m/s            | Calculated vertical speed (positive up). Derived as `-vel_NED_z`.                    |
| `att_NED_roll`                 | radians        | Roll angle (Local Frame). Source: `/mavros/imu/data` (Imu msg, orientation quaternion). |
| `att_NED_pitch`                | radians        | Pitch angle (Local Frame). Source: `/mavros/imu/data` (Imu msg, orientation quaternion).|
| `att_NED_yaw`                  | radians        | Yaw angle (Local Frame). Source: `/tf_simple/yaw` (Float64 msg).                       |
| `attvel_BDY_rollrate`          | rad/s          | Body Frame roll rate. Source: `/mavros/imu/data` (Imu msg, `angular_velocity.x`).      |
| `attvel_BDY_pitchrate`         | rad/s          | Body Frame pitch rate. Source: `/mavros/imu/data` (Imu msg, `angular_velocity.y`).     |
| `attvel_BDY_yawrate`           | rad/s          | Body Frame yaw rate. Source: `/mavros/imu/data` (Imu msg, `angular_velocity.z`).       |

**Notes on Units and Conventions:**
- Radians are used for angles and angular rates unless otherwise specified.
- NED: North, East, Down coordinate system.
- BDY: Body frame of the USV (e.g., X forward, Y left/starboard, Z up). Conventions should be verified with vehicle setup.
- GLB: Global frame, typically WGS84 for Lat/Long/Alt.

### 8.5. Dependencies for `ship_data_logger.py`
- `rospy`
- `numpy`
- `tf.transformations` (optional for roll/pitch from IMU quaternion; if not found, these fields will be blank)
- Standard ROS message types: `geometry_msgs/Twist`, `geometry_msgs/TwistStamped`, `nav_msgs/Odometry`, `sensor_msgs/NavSatFix`, `sensor_msgs/Imu`, `std_msgs/Float64`.

This detailed AGENTS.md should provide a solid foundation for any future work on this repository.
