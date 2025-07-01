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
│       └── franka_ros/       # Franka Panda ROS packages
└── ros1_docker/              # ROS 1 Noetic Docker setup
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
