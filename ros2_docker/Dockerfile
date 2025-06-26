# Use the ROS 2 Humble Long-Term Support (LTS) release as the base image
FROM ros:humble-ros-base

# Install essential system dependencies and tools
# -y flag auto-confirms "yes" to any prompts
RUN apt-get update && apt-get install -y \
    vim \
    iputils-ping \
    net-tools \
    python3-pip \
    dos2unix

# Install required Python packages
RUN pip3 install rospkg jsonpickle

# Set up the ROS workspace environment variable
ENV ROS_WORKSPACE=/ros2_ws

# Create the workspace source directory
RUN mkdir -p $ROS_WORKSPACE/src

# --- File Copying ---
# The paths here are relative to the build context, which should be the project root.

# Copy the ROS 2 package into the workspace's src directory
COPY ./ROS2/src/temp_pkg $ROS_WORKSPACE/src/temp_pkg

# Copy the setup and tutorial scripts into the container's root directory
COPY ./ros2_docker/set-up-workspace /setup.sh
COPY ./ros2_docker/tutorial /tutorial

# --- Script Execution ---

# Convert line endings for all text files in the workspace from Windows (CRLF) to Unix (LF)
# This prevents shell script execution errors inside the Linux container
RUN find $ROS_WORKSPACE -type f -print0 | xargs -0 dos2unix

# Convert scripts copied to root, make them executable, then run the setup script.
# Finally, remove the setup script to keep the final image clean.
RUN dos2unix /tutorial && \
    dos2unix /setup.sh && \
    chmod +x /setup.sh && \
    /setup.sh && \
    rm /setup.sh

# Set the working directory to the ROS workspace
WORKDIR $ROS_WORKSPACE

# Set the entrypoint for the container to the tutorial script
ENTRYPOINT ["/tutorial"]
