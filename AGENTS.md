
# Project LSS Agent Instructions

## Project Overview

This project, named "Program LSS", is a ROS (Robot Operating System) workspace for controlling an autonomous surface vehicle (ASV), likely a boat. It includes packages for autonomy, control, localization, and a ground control station (GCS) with an RViz plugin for visualization and interaction.

## Key Technologies

- **Framework**: ROS (Robot Operating System)
- **Build System**: Catkin
- **Primary Languages**: Python, C++
- **Visualization**: RViz with a custom plugin

## Project Structure

The project follows a standard ROS catkin workspace structure:

- `src/`: Contains the source code for all ROS packages.
- `build/`: Catkin build output folder. (Should be ignored by version control).
- `devel/`: Contains the generated setup files and binaries after a build. (Should be ignored).
- `mission control_*.py`: Top-level mission control scripts.

### Core ROS Packages (`src/`):

- `lss_autonomy`: Handles high-level logic, such as mission control and path following.
- `lss_controller`: Manages low-level vehicle control, like differential drive.
- `lss_gcs`: Ground Control Station components, including map servers and the RViz plugin.
- `lss_localization`: Likely contains nodes for vehicle localization (e.g., using GPS, IMU).
- `lss_srvs`: Defines custom ROS service types for communication between nodes.

## Development Workflows

### 1. Building the Workspace

To build all the packages, navigate to the root of the catkin workspace (`d:/iCloudDrive/ITS/Mutchi/PSP/Program LSS/`) and run:

```bash
catkin_make
```

### 2. Sourcing the Environment

After a successful build, you must source the `setup.bash` file to make the ROS packages and executables available in your terminal environment:

```bash
source devel/setup.bash
```

### 3. Running the System

The primary entry point for launching the entire system seems to be the `all.launch` file.

```bash
roslaunch nala_gcs_bringup all.launch
```

Individual nodes can be run using `rosrun <package_name> <node_name>`.

### 4. Modifying Code

- **Python Nodes**: Modify the Python files directly within the `src` directory of each package (e.g., `src/lss_autonomy/src/mission_control.py`).
- **C++ Nodes**: Modify the C++ source files (e.g., `.cpp`, `.h`) and then rebuild the workspace using `catkin_make`.
- **RViz Plugin**: Changes to the RViz plugin in `lss_gcs/rviz_plugin` will require rebuilding the workspace.

## Code Conventions

- **ROS Best Practices**: Adhere to standard ROS conventions for node naming, topic names, and parameter usage.
- **Python**: Follow PEP 8 for Python code.
- **C++**: Maintain the existing C++ style found in the package you are editing.
- **Commits**: Write clear and concise commit messages explaining the changes made.
