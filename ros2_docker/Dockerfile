# Start from the minimal ROS 2 Humble base image
# This image already contains the correct ROS 2 apt repository sources.
FROM ros:humble-ros-base

# Set the shell to bash for all subsequent RUN commands
SHELL ["/bin/bash", "-c"]

# Prevent interactive prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive

# Install essential system dependencies and RViz2 for visualization
# We do not need to add the ROS 2 repository again as it's in the base image.
RUN apt-get update && apt-get install -y \
    build-essential \
    git \
    python3-pip \
    python3-rosdep \
    python3-colcon-common-extensions \
    ros-humble-rviz2 \
    bash-completion \
    # Add libraries required for Qt applications to connect to X11/WSLg
    libgl1-mesa-glx \
    libglib2.0-0 \
    libxext6 \
    libxrender1 \
    libxtst6 \
    libfontconfig1 \
    && rm -rf /var/lib/apt/lists/*

# The base image may have rosdep initialized, so we only need to update it.
RUN rosdep update || true

# Create the Franka workspace
ENV FRANKA_WS=/franka_ws
WORKDIR $FRANKA_WS/src

# Clone the required repositories
# We are NOT cloning panda_moveit_config for now.
RUN git clone https://github.com/frankaemika/franka_description.git
RUN git clone https://github.com/frankaemika/franka_ros2.git -b humble

# Install all dependencies for the cloned packages
# We run `apt-get update` again here to ensure the package lists are fresh before rosdep runs.
WORKDIR $FRANKA_WS
RUN apt-get update && rosdep install --from-paths src --ignore-src -r -y --rosdistro humble

# Build the workspace
# Source the base ROS 2 setup file before building
RUN source /opt/ros/humble/setup.bash && colcon build --symlink-install

# Activate ROS 2 tab completion globally for the root user's bash shell
RUN echo "source /usr/share/bash-completion/completions/git" >> /root/.bashrc
RUN echo "source /opt/ros/humble/share/ros2cli/environment/ros2-argcomplete.bash" >> /root/.bashrc

# --- Final Setup ---
# Copy the entrypoint script into the container and make it executable
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Set the entrypoint for the container
ENTRYPOINT ["/entrypoint.sh"]

# The default command to run when the container starts is a bash shell
CMD ["bash"]
