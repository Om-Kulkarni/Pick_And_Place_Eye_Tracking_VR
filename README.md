# Pick_And_Place_Eye_Tracking_VR
An eye tracking VR project where users can select targets for robot pick-and-place operations using eye tracking technology. The project integrates the Franka Emika Panda robot with Unity VR for immersive robotic control.

## Features
- Eye tracking-based target selection in VR
- Franka Panda robot simulation and control
- ROS 1 integration with Unity
- Docker-based development environment
- Pick and place task execution

## Project Structure

```
Pick_And_Place_Eye_Tracking_VR/
├── PickAndPlaceVR/           # Unity VR Project
├── ROS/
│   └── src/
│       ├── franka_ros/       # Franka Panda ROS packages
│       ├── panda_moveit_config/ # MoveIt configuration for Panda
│       └── ros_tcp_endpoint/ # Unity-ROS communication bridge
└── ros1_docker/              # ROS 1 Noetic Docker setup
```

## Setup Instructions

### Prerequisites
- Docker Desktop with WSL2 backend
- Git (for cloning repositories)
- Windows 11 (for WSLg GUI support)

### Clone Required ROS Packages

Before building the Docker image, you need to clone the required ROS packages into your `ROS/src` directory:

```bash
# Navigate to the ROS source directory
cd ROS/src/

# Clone Franka ROS packages
git clone git@github.com:frankarobotics/franka_ros.git

# Clone Panda MoveIt configuration
git clone git@github.com:moveit/panda_moveit_config.git

# Clone Unity ROS TCP Endpoint for Unity-ROS communication
git clone git@github.com:Unity-Technologies/ROS-TCP-Endpoint.git ros_tcp_endpoint
```

**Alternative (if SSH keys not configured):**
If you don't have SSH keys set up with GitHub, use HTTPS URLs instead:
```bash
cd ROS/src/
git clone https://github.com/frankarobotics/franka_ros.git
git clone https://github.com/moveit/panda_moveit_config.git
git clone https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git ros_tcp_endpoint
```

**Note**: The last clone command renames the repository from `ROS-TCP-Endpoint` to `ros_tcp_endpoint` to match ROS naming conventions.

### Troubleshooting

### Troubleshooting

**Line Ending Issues (ros_tcp_endpoint):**
The `ros_tcp_endpoint` package from GitHub may have Windows-style (CRLF) line endings and incorrect Python shebang lines that can cause errors like `/usr/bin/env: 'python\r': No such file or directory` or `/usr/bin/env: 'python': No such file or directory`. **This is now automatically handled by the Docker build process**, which converts all Python files to Unix-style (LF) line endings and updates shebang lines to use `python3` during the container build.

If you need to manually fix these issues outside of Docker, you can:

1. **Re-clone with proper line endings:**
   ```bash
   # Remove the problematic directory
   rm -rf ROS/src/ros_tcp_endpoint
   
   # Configure Git to handle line endings
   git config core.autocrlf input
   
   # Re-clone the repository
   cd ROS/src/
   git clone https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git ros_tcp_endpoint
   ```

2. **Or fix existing files:**
   ```bash
   # Convert line endings and fix shebang lines in Python files
   find ROS/src/ros_tcp_endpoint -name "*.py" -exec dos2unix {} \;
   find ROS/src/ros_tcp_endpoint -name "*.py" -exec sed -i '1s|^#!/usr/bin/env python$|#!/usr/bin/env python3|' {} \;
   ```

   **Note**: You may need to install `dos2unix` if it's not available:
   ```bash
   # On Ubuntu/Debian
   apt-get update && apt-get install -y dos2unix
   
   # Then run the conversion
   find ROS/src/ros_tcp_endpoint -name "*.py" -exec dos2unix {} \;
   find ROS/src/ros_tcp_endpoint -name "*.py" -exec sed -i '1s|^#!/usr/bin/env python$|#!/usr/bin/env python3|' {} \;
   ```

## Docker Instructions

### ROS 1 Noetic (Franka Panda)

1. **Setup the Workspace**
   Ensure you have cloned the required ROS packages (see "Clone Required ROS Packages" section above) into your `ROS/src` directory:
   - `franka_ros/` - Franka Panda ROS packages
   - `panda_moveit_config/` - MoveIt configuration for Panda
   - `ros_tcp_endpoint/` - Unity-ROS communication bridge
   
   The Docker build will use these local copies.

2. **Build the Docker Image**
   ```bash
   # Build from the project root directory
   docker build -t franka-ros-noetic -f ros1_docker/Dockerfile .
   ```

3. **Start the Docker Container**
   Start a detached container with GUI support and port forwarding:
   ```powershell
   docker run -d --name franka_container `
                -it `
                -p 10000:10000 `
                -p 5005:5005 `
                -v /run/desktop/mnt/host/wslg/.X11-unix:/tmp/.X11-unix `
                -v /run/desktop/mnt/host/wslg:/mnt/wslg `
                -e DISPLAY=:0 `
                -e WAYLAND_DISPLAY=wayland-0 `
                -e XDG_RUNTIME_DIR=/mnt/wslg/runtime-dir `
                -e PULSE_SERVER=/mnt/wslg/PulseServer `
                franka-ros-noetic `
                tail -f /dev/null
   ```

   **Port forwarding explained:**
   - `-p 10000:10000`: Unity-ROS TCP communication port
   - `-p 5005:5005`: Additional communication/debugging port

4. **Start ROS TCP Endpoint (Unity-ROS Communication Bridge)**
   In a new terminal, launch the TCP endpoint that handles communication between ROS and Unity:
   ```bash
   # Enter the container
   docker exec -it franka_container /bin/bash

   # Inside the container:
   . devel/setup.bash
   roslaunch ros_tcp_endpoint endpoint.launch
   ```

   This will start the ROS TCP endpoint server on port 10000, allowing Unity to communicate with ROS nodes. The `roslaunch` command will automatically start `roscore` if it's not already running.

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

5. **Generate ROS Messages for Franka**
   - In Unity, go to `Robotics -> Generate ROS Messages`
   - Navigate to your ROS workspace: `ROS/src/franka_ros/franka_msgs/`
   - Select the message types you need (typically):
     - `FrankaState.msg` - For robot state information
     - `Errors.msg` - For error handling
     - Any other Franka-specific messages required for your application
   - Click "Build" to generate the C# message classes in `Assets/RosMessages/`

6. **Configure Message Browser Settings**
   - The message browser will create a `msgbrowser_settings.asset` file with your local paths
   - This file is ignored by Git since it contains user-specific absolute paths
   - Each developer needs to configure their own paths through Unity's Robotics tools

**Note**: The `franka_panda` folder, `RosMessages` folder, and `msgbrowser_settings.asset` are included in `.gitignore` since they contain generated files and user-specific settings that should be built locally using Unity's tools.

## Starting the Unity Simulation

Once you have set up the ROS environment and Unity project, follow these steps to run the pick-and-place simulation:

### Prerequisites
- Unity scene is set up with the Franka Panda robot
- Docker container is built and ready
- Unity project has the PandaTrajectoryPlanner script configured

### Step-by-Step Launch Process

1. **Start the Docker Container**
   ```powershell
   # Start the container if not already running
   docker run -d --name franka_container `
                -it `
                -p 10000:10000 `
                -p 5005:5005 `
                -v /run/desktop/mnt/host/wslg/.X11-unix:/tmp/.X11-unix `
                -v /run/desktop/mnt/host/wslg:/mnt/wslg `
                -e DISPLAY=:0 `
                -e WAYLAND_DISPLAY=wayland-0 `
                -e XDG_RUNTIME_DIR=/mnt/wslg/runtime-dir `
                -e PULSE_SERVER=/mnt/wslg/PulseServer `
                franka-ros-noetic `
                tail -f /dev/null
   ```

2. **Launch the Panda MoveIt Service**
   Open a terminal and enter the Docker container:
   ```bash
   # Enter the container
   docker exec -it franka_container /bin/bash

   # Inside the container, launch the Panda MoveIt integration
   . devel/setup.bash
   roslaunch panda_moveit panda_unity_integration.launch
   ```

   This will start:
   - ROS core
   - MoveIt planning server for the Panda robot
   - Unity-ROS TCP communication endpoint
   - Panda trajectory planning service

3. **Start Unity**
   - Open the Unity project (`PickAndPlaceVR`)
   - Load the main scene with the Panda robot setup
   - Ensure the `PandaTrajectoryPlanner` script is attached to the appropriate GameObject
   - Check that the robot, target, and placement objects are properly assigned in the Inspector

4. **Connect and Execute Pick-and-Place**
   - In Unity, press Play to start the simulation
   - Look for the "Publish" button at the bottom of the Game view/screen
   - Set up your target object and placement location in the scene
   - Click the **"Publish" button** to execute the pick-and-place operation

### What Happens When You Press "Publish"

The "Publish" button triggers the `ExecutePickAndPlace()` method which:
1. Checks if targets are properly set
2. Calculates optimal pick and place poses based on object orientation
3. Sends trajectory planning requests to the ROS MoveIt service
4. Executes the planned trajectories point-by-point
5. Controls the gripper to pick up and place the object

### Troubleshooting

**If the simulation doesn't start:**
- Verify the Docker container is running: `docker ps`
- Check ROS services are active: `docker exec -it franka_container rosnode list`
- Ensure Unity can connect to ROS (check Unity Console for connection messages)

**If pick-and-place fails:**
- Check that target objects have appropriate physics properties (Rigidbody, reasonable mass)
- Verify gripper positions in the Inspector (`GripperOpenPosition`, `GripperClosedPosition`)
- Monitor the Unity Console for trajectory planning errors
- Ensure objects are within the robot's workspace

**Performance Tips:**
- Adjust `SpeedFactor` in the PandaTrajectoryPlanner to control execution speed
- Modify `PickOffsetHeight` and `GripperTcpOffset` for different object sizes
- Check object physics materials for proper friction and collision properties
