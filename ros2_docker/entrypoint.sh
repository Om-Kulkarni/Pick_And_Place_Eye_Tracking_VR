#!/bin/bash
# This entrypoint script is executed when the Docker container starts.

# Source the ROS 2 Humble installation to set up the environment
source /opt/ros/humble/setup.bash

# Source the local Franka workspace to make its packages available
source /franka_ws/install/setup.bash

# Execute any command passed to the 'docker run' command.
# If no command is passed, it will default to the CMD in the Dockerfile (bash).
exec "$@"