# Pick_And_Place_Eye_Tracking_VR
A eye tracking project where the user can choose the target that the robot should pick and the destination where it should be placed using Eye Tracking

## Currently working on configuring franka panda for this task instead of the niryo

## Project Structure

```
Pick_And_Place_Eye_Tracking_VR/
├── ROS/
│   └── src/
│       └── franka_ros/        # Franka Panda ROS packages
├── ros1_docker/              # ROS 1 Noetic Docker setup
└── ros2_docker/              # ROS 2 Humble Docker setup
```

## Docker Instructions

### ROS 1 Noetic (Franka Panda)

1. **Setup the Workspace**
   First, ensure you have the Franka ROS package in your `ROS/src` directory. The Docker build will use this local copy instead of downloading from GitHub.

2. **Build the Docker Image**
   ```bash
   # Build from the project root directory
   docker build -t franka-ros-noetic -f ros1_docker/Dockerfile .
   ```

3. **Start the Docker Container**
   Start a detached container with GUI support:
   ```powershell
   docker run -d --name franka_container `
                -it `
                -v /run/desktop/mnt/host/wslg/.X11-unix:/tmp/.X11-unix `
                -v /run/desktop/mnt/host/wslg:/mnt/wslg `
                -e DISPLAY=:0 `
                -e WAYLAND_DISPLAY=wayland-0 `
                -e XDG_RUNTIME_DIR=/mnt/wslg/runtime-dir `
                -e PULSE_SERVER=/mnt/wslg/PulseServer `
                franka-ros-noetic `
                tail -f /dev/null
   ```

4. **Start ROS Core**
   In a new terminal, execute:
   ```bash
   # Enter the container
   docker exec -it franka_container /bin/bash

   # Inside the container:
   . devel/setup.bash
   roscore
   ```

5. **Launch Robot Visualization**
   In another new terminal:
   ```bash
   # Enter the container
   docker exec -it franka_container /bin/bash

   # Inside the container:
   . devel/setup.bash
   roslaunch franka_description display.launch
   ```

   Note: For connecting to a real robot, use `roslaunch franka_visualization franka_visualization.launch robot_ip:=<ROBOT_IP>` instead. 
   If you see libfranka connection errors with the visualization, this is normal when no real robot is connected and can be ignored for pure visualization purposes.

### ROS 2 Humble (Franka Panda)

This project also provides a Dockerized environment for simulating the Franka Emika Panda robot with ROS 2 Humble and MoveIt 2. It is designed to work on Windows with Docker Desktop and WSL2.

## Prerequisites

Before you begin, ensure you have the following installed and configured on your Windows machine:

1.  **Windows 11:** Required for the WSLg feature which provides GUI support.
2.  **Docker Desktop:** The latest version, configured to use the WSL 2 backend.
3.  **WSL 2:** Ensure you have a WSL 2 distribution installed (e.g., Ubuntu).

## 1. Build the Docker Image

This command builds the Docker image from the `Dockerfile`. It will download the ROS 2 desktop environment, clone the necessary Franka Panda repositories, install all dependencies, and build the ROS 2 workspace. This process will take some time.

Open a PowerShell terminal, navigate to your `franka_docker` directory, and run:

```bash
docker build -t franka-ros2-humble .
```

## 2. Run the Docker Container

This command starts the container and correctly configures it to forward GUI applications (like RViz2) to your Windows desktop using WSLg. It works by mounting the special WSLg sockets and setting the required environment variables.

Run this command in your PowerShell terminal:

```powershell
docker run -it -v /run/desktop/mnt/host/wslg/.X11-unix:/tmp/.X11-unix `
                -v /run/desktop/mnt/host/wslg:/mnt/wslg `
                -e DISPLAY=:0 `
                -e WAYLAND_DISPLAY=wayland-0 `
                -e XDG_RUNTIME_DIR=/mnt/wslg/runtime-dir `
                -e PULSE_SERVER=/mnt/wslg/PulseServer `
                franka-ros2-humble
```
This will drop you into a `bash` shell inside the running container.

## 3. Launch the Robot Visualization

start all the necessary nodes and open RViz2.

From the container's command prompt, run:

```bash
ros2 launch franka_description visualize_franka.launch.py arm_id:=fp3
```
You should now see the RViz2 window appear on your desktop, displaying the Franka Panda robot.
