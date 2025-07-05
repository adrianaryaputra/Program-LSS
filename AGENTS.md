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

This detailed AGENTS.md should provide a solid foundation for any future work on this repository.
