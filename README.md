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

5. **Launch Robot Visualization with MoveIt**
   In another new terminal:
   ```bash
   # Enter the container
   docker exec -it franka_container /bin/bash

   # Inside the container:
   . devel/setup.bash
   roslaunch panda_moveit_config demo.launch
   ```

   This will launch:
   - RViz with MoveIt plugin
   - Motion Planning capabilities
   - Interactive markers for robot control
   - Collision detection and path planning

6. **Gazebo Simulation and Gripper Control**
   To test pick and place operations in Gazebo:

   a. Launch Gazebo simulation with RViz:
   ```bash
   # Enter the container
   docker exec -it franka_container /bin/bash

   # Inside the container:
   . devel/setup.bash
   roslaunch franka_gazebo panda.launch x:=-0.5 \
       world:=$(rospack find franka_gazebo)/world/stone.sdf \
       controller:=cartesian_impedance_example_controller \
       rviz:=true
   ```

   b. Control the gripper:
   ```bash
   # Open gripper (width: 0.08 meters, speed: 0.1 m/s)
   rostopic pub --once /franka_gripper/move/goal franka_gripper/MoveActionGoal "goal: { width: 0.08, speed: 0.1 }"

   # Grasp object (width: 0.03 meters, force: 5N)
   rostopic pub --once /franka_gripper/grasp/goal \
       franka_gripper/GraspActionGoal \
       "goal: { width: 0.03, epsilon:{ inner: 0.005, outer: 0.005 }, speed: 0.1, force: 5.0}"
   ```

   The grasp command parameters:
   - `width`: Target width (meters)
   - `epsilon`: Tolerances for grasp width
   - `speed`: Closing speed (m/s)
   - `force`: Grasping force (Newtons)

### Unity URDF Setup

To use the Franka Panda robot in Unity, you need to generate a URDF file from the Xacro and copy the necessary mesh files:

1. **Copy Franka Description Package**
   First, copy the `franka_description` package from your ROS workspace to the Unity URDF folder:
   ```bash
   # Navigate to your Unity project
   cd PickAndPlaceVR/Assets/URDF/franka_panda/
   
   # Copy the franka_description package from ROS workspace
   cp -r ../../../ROS/src/franka_ros/franka_description/ ./
   ```

2. **Generate URDF from Xacro in Docker Container**
   Enter the Docker container and generate the URDF:
   ```bash
   # Enter the container
   docker exec -it franka_container /bin/bash

   # Inside the container, navigate to franka_description
   cd /catkin_ws/src/franka_ros/franka_description/

   # Generate the URDF file with Gazebo and hand enabled
   rosrun xacro xacro robots/panda/panda.urdf.xacro gazebo:=true hand:=true > panda.urdf
   ```

3. **Copy URDF from Container to Unity**
   From your host machine (outside the container), copy the generated URDF:
   ```bash
   # Copy the generated URDF from container to Unity project
   docker cp franka_container:/catkin_ws/src/franka_ros/franka_description/panda.urdf ./PickAndPlaceVR/Assets/URDF/franka_panda/

   # Ensure mesh paths are correct for Unity (should be relative paths like franka_description/meshes/...)
   # The URDF should already have the correct paths after generation
   ```

4. **Import in Unity**
   - Open Unity and navigate to `Assets/URDF/franka_panda/`
   - Select the `panda.urdf` file
   - Use Unity's URDF Importer to import the robot
   - The mesh files should be automatically located in `franka_description/meshes/`

**Note**: The `franka_panda` folder is included in `.gitignore` since it contains generated files and large mesh assets that should be built locally.

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
