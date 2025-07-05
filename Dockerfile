# 1. Base Image
FROM ros:noetic-ros-base-focal

# 2. Environment Setup
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=noetic
ENV CATKIN_WS=/catkin_ws
# Set shell to bash for RUN commands to ensure complex commands work as expected
SHELL ["/bin/bash", "-c"]

# 3. System & Build Dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    python3-catkin-tools \
    python3-rosdep \
    python3-pip \
    ros-noetic-cv-bridge \ # For packages using OpenCV via cv_bridge
    ros-noetic-rviz \      # For RVIZ and its development files for plugins
    ros-noetic-mavros \    # For MAVROS nodes and messages
    ros-noetic-mavros-msgs \ # Explicitly for MAVROS messages
    ros-noetic-tf \        # For TF transformations library
    # Add any other core system libraries that might be discovered as missing during build
    && rm -rf /var/lib/apt/lists/*

# 4. Python Dependencies (pip)
# numpy is used by ship_data_logger.py
# pyserial is used by lss_controller/src/differential_control.py
RUN pip3 install --no-cache-dir \
    numpy \
    pyserial

# 5. Create Catkin Workspace, Initialize rosdep, Copy Code
RUN mkdir -p ${CATKIN_WS}/src
WORKDIR ${CATKIN_WS}/src

# Copy the entire repository content into a subdirectory within src.
# This assumes the Dockerfile is at the root of your project repository.
COPY . ./usv_project_source/

# Initialize rosdep and install dependencies for the copied source code
WORKDIR ${CATKIN_WS}
RUN rosdep init && rosdep update
# Install ROS dependencies based on package.xml files in the source directory
# The path points to the directory containing all the packages to be built.
RUN rosdep install --from-paths src/usv_project_source \
    --ignore-src \
    -r -y \
    --rosdistro ${ROS_DISTRO}

# 6. Build Catkin Workspace
# Source ROS setup before building to ensure catkin can find ROS packages
# catkin build will build all packages found in CATKIN_WS/src/*
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    catkin build -DCMAKE_BUILD_TYPE=Release

# 7. Setup entrypoint to source workspace
# The base image's /ros_entrypoint.sh sources /opt/ros/$ROS_DISTRO/setup.bash.
# We add sourcing of our workspace's devel space to .bashrc for interactive shells.
RUN echo "source ${CATKIN_WS}/devel/setup.bash" >> ~/.bashrc

# The ship_data_logger.py and test_ship_data_logger.py are now at:
# ${CATKIN_WS}/src/usv_project_source/ship_data_logger.py
# ${CATKIN_WS}/src/usv_project_source/test_ship_data_logger.py
# They can be executed using their full paths or relative paths if WORKDIR is set accordingly.
# To make them executable as ROS nodes, they should be part of a package's scripts dir
# and have executable permissions, or be launched via roslaunch which can find them if
# they are in a package.

# 8. Set Default Working Directory (Optional, for convenience)
# Setting it to the root of the copied project source.
WORKDIR ${CATKIN_WS}/src/usv_project_source

# 9. Default Command
# Provides an interactive bash shell when the container starts.
# The ROS environment and the workspace will be sourced due to .bashrc and ros_entrypoint.sh.
CMD ["bash"]
