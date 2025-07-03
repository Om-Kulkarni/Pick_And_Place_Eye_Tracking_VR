using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using Unity.Robotics.UrdfImporter;
using RosMessageTypes.PandaMoveit;  // Updated to use PandaMoveit messages
using RosMessageTypes.Geometry;
using RosMessageTypes.Trajectory;
using RosMessageTypes.Moveit;       // For RobotTrajectoryMsg

namespace Unity.Robotics.PickAndPlace
{
    /// <summary>
    ///     Trajectory planner for the Franka Panda robot using MoveIt
    /// </summary>
    public class PandaTrajectoryPlanner : MonoBehaviour
    {
        const int k_NumRobotJoints = 7;
        const string k_RosServiceName = "panda_moveit";
        
        public static readonly string[] LinkNames =
        {
            "world/panda_link0/panda_link1", "/panda_link2", "/panda_link3", "/panda_link4", 
            "/panda_link5", "/panda_link6", "/panda_link7"
        };

        [SerializeField]
        string m_RosServiceName = "panda_moveit";
        public string RosServiceName { get => m_RosServiceName; set => m_RosServiceName = value; }

        [SerializeField]
        [Tooltip("The GameObject representing the Panda robot")]
        GameObject m_PandaRobot;
        public GameObject PandaRobot { get => m_PandaRobot; set => m_PandaRobot = value; }
        [SerializeField]
        GameObject m_Target;
        public GameObject Target { get => m_Target; set => m_Target = value; }
        [SerializeField]
        GameObject m_TargetPlacement;
        public GameObject TargetPlacement { get => m_TargetPlacement; set => m_TargetPlacement = value; }

        [SerializeField]
        [Tooltip("Speed factor for trajectory execution (0.1 = 10% speed)")]
        float m_SpeedFactor = 0.5f;

        [SerializeField]
        [Tooltip("Duration for each trajectory segment")]
        float m_TrajectoryExecutionTime = 5.0f;

        [SerializeField]
        [Tooltip("Height above target for approach (in meters)")]
        float m_PickOffsetHeight = 0.1f;

        [SerializeField]
        [Tooltip("MoveIt planning group to use for trajectory planning")]
        string m_PlanningGroup = "panda_manipulator";
        public string PlanningGroup => m_PlanningGroup;

        // Assures that the gripper is always positioned above the target before grasping (like Niryo)
        Vector3 m_PickPoseOffset => Vector3.up * m_PickOffsetHeight;
        
        // Hardcoded pick orientation to ensure gripper approaches from above (like Niryo's approach)
        readonly Quaternion m_PickOrientation = Quaternion.Euler(180, 0, 0); // Gripper pointing down

        // Robot joints
        UrdfJointRevolute[] m_JointArticulationBodies;
        
        // Gripper ArticulationBodies (similar to Niryo)
        ArticulationBody m_LeftGripper;
        ArticulationBody m_RightGripper;
        
        // ROS connection
        ROSConnection m_Ros;

        // State tracking
        bool m_IsExecutingTrajectory = false;
        bool m_IsPickAndPlaceComplete = false;

        public bool IsExecutingTrajectory => m_IsExecutingTrajectory;
        public bool IsPickAndPlaceComplete => m_IsPickAndPlaceComplete;

        void Start()
        {
            // Get ROS connection
            m_Ros = ROSConnection.GetOrCreateInstance();
            m_Ros.RegisterRosService<PandaMoverServiceRequest, PandaMoverServiceResponse>(k_RosServiceName);

            // Check if robot is assigned
            if (m_PandaRobot == null)
            {
                Debug.LogError("PandaRobot GameObject is not assigned! Please assign it in the Inspector.");
                return;
            }

            // Initialize joint references
            m_JointArticulationBodies = new UrdfJointRevolute[k_NumRobotJoints];
            var linkName = string.Empty;
            for (var i = 0; i < k_NumRobotJoints; i++)
            {
                linkName += LinkNames[i];
                m_JointArticulationBodies[i] = m_PandaRobot.transform.Find(linkName).GetComponent<UrdfJointRevolute>();
            }

            // Find Panda gripper fingers (similar to Niryo approach)
            // Adjust these paths based on your Panda robot's gripper structure
            var leftGripperPath = linkName + "/panda_link8/panda_hand/panda_leftfinger";
            var rightGripperPath = linkName + "/panda_link8/panda_hand/panda_rightfinger";
            
            var leftGripperTransform = m_PandaRobot.transform.Find(leftGripperPath);
            var rightGripperTransform = m_PandaRobot.transform.Find(rightGripperPath);
            
            if (leftGripperTransform != null)
                m_LeftGripper = leftGripperTransform.GetComponent<ArticulationBody>();
            if (rightGripperTransform != null)
                m_RightGripper = rightGripperTransform.GetComponent<ArticulationBody>();
                
            if (m_LeftGripper == null || m_RightGripper == null)
            {
                Debug.LogWarning("Panda gripper ArticulationBodies not found. Please check gripper paths.");
            }

            Debug.Log("PandaTrajectoryPlanner initialized successfully");
        }

        /// <summary>
        ///     Execute a full pick and place operation
        /// </summary>
        public void ExecutePickAndPlace()
        {
            Debug.Log("ExecutePickAndPlace() method called!");
            
            if (m_IsExecutingTrajectory)
            {
                Debug.LogWarning("Already executing trajectory, please wait");
                return;
            }

            if (m_Target == null || m_TargetPlacement == null)
            {
                Debug.LogError("Pick and place targets must be set");
                return;
            }

            Debug.Log("Starting pick and place coroutine...");
            StartCoroutine(ExecutePickAndPlaceCoroutine());
        }

        /// <summary>
        ///     Execute pick and place operation as a coroutine
        /// </summary>
        IEnumerator ExecutePickAndPlaceCoroutine()
        {
            m_IsExecutingTrajectory = true;
            m_IsPickAndPlaceComplete = false;

            Debug.Log("Starting pick and place operation...");

            // Create service request
            var request = new PandaMoverServiceRequest();
            request.joints_input = GetCurrentJointState();
            
            // Pick Pose (with offset like Niryo)
            // Convert Unity world positions to robot-relative positions
            var pickPosition = m_Target.transform.position + m_PickPoseOffset;
            var pickPositionRelativeToRobot = pickPosition - m_PandaRobot.transform.position;
            var pickPose = new PoseMsg
            {
                position = pickPositionRelativeToRobot.To<FLU>(),
                // Use hardcoded orientation to ensure gripper approaches from above (like Niryo)
                orientation = m_PickOrientation.To<FLU>()
            };
            request.pick_pose = pickPose;
            
            // Place Pose (with offset like Niryo)
            // Convert Unity world positions to robot-relative positions
            var placePosition = m_TargetPlacement.transform.position + m_PickPoseOffset;
            var placePositionRelativeToRobot = placePosition - m_PandaRobot.transform.position;
            var placePose = new PoseMsg
            {
                position = placePositionRelativeToRobot.To<FLU>(),
                // Use hardcoded orientation for consistent placement (like Niryo)
                orientation = m_PickOrientation.To<FLU>()
            };
            request.place_pose = placePose;

            // Send service request
            bool serviceCallComplete = false;
            PandaMoverServiceResponse response = null;

            m_Ros.SendServiceMessage<PandaMoverServiceResponse>(k_RosServiceName, request, (r) =>
            {
                response = r;
                serviceCallComplete = true;
            });

            // Wait for service response
            yield return new WaitUntil(() => serviceCallComplete);

            if (response == null || response.trajectories.Length == 0)
            {
                Debug.LogError("No trajectory returned from MoverService.");
                m_IsExecutingTrajectory = false;
                yield break;
            }

            Debug.Log($"Received {response.trajectories.Length} trajectories from MoveIt");

            // Execute trajectories
            yield return StartCoroutine(ExecuteTrajectories(response));

            m_IsExecutingTrajectory = false;
            m_IsPickAndPlaceComplete = true;
            Debug.Log("Pick and place operation completed successfully");
        }

        /// <summary>
        ///     Execute the trajectories returned by MoveIt (following Niryo pattern)
        /// </summary>
        /// <param name="response">PandaMoverServiceResponse containing trajectory plans</param>
        IEnumerator ExecuteTrajectories(PandaMoverServiceResponse response)
        {
            if (response.trajectories != null)
            {
                // For every trajectory plan returned (like Niryo)
                for (var trajectoryIndex = 0; trajectoryIndex < response.trajectories.Length; trajectoryIndex++)
                {
                    var trajectory = response.trajectories[trajectoryIndex];
                    
                    Debug.Log($"Executing trajectory {trajectoryIndex + 1}/{response.trajectories.Length}");

                    // Execute the trajectory
                    yield return StartCoroutine(ExecuteTrajectory(trajectory));
                    
                    // Close the gripper if completed executing the trajectory for the Grasp pose (like Niryo)
                    if (trajectoryIndex == (int)Poses.Grasp)
                    {
                        Debug.Log("Closing gripper...");
                        yield return StartCoroutine(HandleGripperAction(true));
                    }
                    
                    // Wait for the robot to achieve the final pose from joint assignment (like Niryo)
                    yield return new WaitForSeconds(0.5f / m_SpeedFactor);
                }

                // All trajectories have been executed, open the gripper to place the target cube (like Niryo)
                Debug.Log("Opening gripper...");
                yield return StartCoroutine(HandleGripperAction(false));
            }
        }
        
        enum Poses
        {
            PreGrasp,
            Grasp,
            PickUp,
            Place
        }

        /// <summary>
        ///     Execute a single trajectory (using Niryo-style approach)
        /// </summary>
        /// <param name="trajectory">The trajectory to execute</param>
        IEnumerator ExecuteTrajectory(RobotTrajectoryMsg trajectory)
        {
            if (trajectory.joint_trajectory.points.Length == 0)
            {
                Debug.LogWarning("Empty trajectory received");
                yield break;
            }

            var jointTrajectory = trajectory.joint_trajectory;
            
            // Execute each trajectory point sequentially (like Niryo)
            foreach (var trajectoryPoint in jointTrajectory.points)
            {
                var jointPositions = trajectoryPoint.positions;
                
                // Set the joint values for every joint (convert radians to degrees for Unity)
                for (var joint = 0; joint < k_NumRobotJoints && joint < jointPositions.Length; joint++)
                {
                    var jointAngleDegrees = (float)jointPositions[joint] * Mathf.Rad2Deg;
                    SetJointPosition(joint, jointAngleDegrees);
                }

                // Wait for robot to achieve pose for all joint assignments (like Niryo)
                yield return new WaitForSeconds(0.1f / m_SpeedFactor); // Apply speed factor to wait time
            }

            Debug.Log("Trajectory execution completed");
        }
        
        /// <summary>
        ///     Handle gripper open/close actions using ArticulationBody control (Niryo-style)
        /// </summary>
        /// <param name="close">True to close gripper, false to open</param>
        IEnumerator HandleGripperAction(bool close)
        {
            if (m_LeftGripper != null && m_RightGripper != null)
            {
                // Real ArticulationBody gripper control (like Niryo)
                var leftDrive = m_LeftGripper.xDrive;
                var rightDrive = m_RightGripper.xDrive;

                if (close)
                {
                    // Close gripper - adjust values based on your Panda gripper configuration
                    leftDrive.target = 0.0f;   // Adjust these values for Panda gripper
                    rightDrive.target = 0.0f;  // Adjust these values for Panda gripper
                    Debug.Log("Closing Panda gripper...");
                }
                else
                {
                    // Open gripper - adjust values based on your Panda gripper configuration
                    leftDrive.target = 0.04f;  // Adjust these values for Panda gripper
                    rightDrive.target = 0.04f; // Adjust these values for Panda gripper
                    Debug.Log("Opening Panda gripper...");
                }

                m_LeftGripper.xDrive = leftDrive;
                m_RightGripper.xDrive = rightDrive;

                yield return new WaitForSeconds(0.5f); // Wait for gripper to move
            }
            else
            {
                // Fallback to simulated approach if gripper ArticulationBodies not found
                Debug.LogWarning("Gripper ArticulationBodies not available, using simulated gripper");
                
                if (close && m_Target != null)
                {
                    // Attach object to gripper at the TCP (Tool Center Point) of panda_manipulator
                    var endEffector = GetEndEffectorTransform();
                    if (endEffector != null)
                    {
                        m_Target.transform.SetParent(endEffector);
                        m_Target.transform.localPosition = Vector3.zero;
                        Debug.Log("Object attached to end-effector (simulated)");
                    }
                    else
                    {
                        Debug.LogError("Could not find end-effector transform to attach object");
                    }
                }
                else if (!close && m_Target != null)
                {
                    // Detach object from gripper
                    m_Target.transform.SetParent(null);
                    Debug.Log("Object detached from gripper (simulated)");
                }

                yield return new WaitForSeconds(0.5f);
            }
        }

        /// <summary>
        ///     Get current joint state for the service request
        /// </summary>
        PandaMoveitJointsMsg GetCurrentJointState()
        {
            var joints = new PandaMoveitJointsMsg();
            var positions = GetCurrentJointPositions();
            joints.joints = new double[k_NumRobotJoints];  // Note: using double array for 7 DOF
            
            for (int i = 0; i < k_NumRobotJoints; i++)
            {
                joints.joints[i] = positions[i];  // Keep as double for precision
            }

            return joints;
        }

        /// <summary>
        ///     Get current joint positions
        /// </summary>
        double[] GetCurrentJointPositions()
        {
            var positions = new double[k_NumRobotJoints];
            for (int i = 0; i < k_NumRobotJoints; i++)
            {
                // Get position from ArticulationBody and convert from degrees to radians if needed
                var articulationBody = m_JointArticulationBodies[i].GetComponent<ArticulationBody>();
                if (articulationBody != null)
                {
                    // Unity ArticulationBody positions are typically in degrees, convert to radians for ROS
                    positions[i] = articulationBody.jointPosition[0] * Mathf.Deg2Rad;
                }
                else
                {
                    // Fallback to URDF joint method if available
                    positions[i] = m_JointArticulationBodies[i].GetPosition();
                }
            }
            return positions;
        }

        /// <summary>
        ///     Set position for a specific joint (Niryo-style approach)
        /// </summary>
        void SetJointPosition(int jointIndex, float positionDegrees)
        {
            if (jointIndex < k_NumRobotJoints)
            {
                var articulationBody = m_JointArticulationBodies[jointIndex].GetComponent<ArticulationBody>();
                if (articulationBody != null)
                {
                    var xDrive = articulationBody.xDrive;
                    xDrive.target = positionDegrees; // Already in degrees, no conversion needed
                    articulationBody.xDrive = xDrive;
                }
            }
        }

        /// <summary>
        ///     Convert Unity Transform to ROS Pose message (relative to robot base)
        /// </summary>
        PoseMsg GetPoseMsg(Transform transform)
        {
            var pose = new PoseMsg();
            
            // Convert Unity position to robot-relative position, then to ROS using FLU coordinate system
            var relativePosition = transform.position - m_PandaRobot.transform.position;
            pose.position = relativePosition.To<FLU>();

            // Convert Unity quaternion to ROS using FLU coordinate system
            pose.orientation = transform.rotation.To<FLU>();

            return pose;
        }

        /// <summary>
        ///     Move to a specific pose using MoveIt (for direct control)
        /// </summary>
        public void MoveToPose(Transform targetPose)
        {
            if (m_IsExecutingTrajectory)
            {
                Debug.LogWarning("Already executing trajectory, please wait");
                return;
            }

            StartCoroutine(MoveToPoseCoroutine(targetPose));
        }

        /// <summary>
        ///     Move to pose coroutine
        /// </summary>
        IEnumerator MoveToPoseCoroutine(Transform targetPose)
        {
            m_IsExecutingTrajectory = true;

            // Create a simple pick-and-place request where pick and place poses are the same
            var request = new PandaMoverServiceRequest();
            request.joints_input = GetCurrentJointState();
            request.pick_pose = GetPoseMsg(targetPose);
            request.place_pose = GetPoseMsg(targetPose);

            // Send service request
            bool serviceCallComplete = false;
            PandaMoverServiceResponse response = null;

            m_Ros.SendServiceMessage<PandaMoverServiceResponse>(k_RosServiceName, request, (r) =>
            {
                response = r;
                serviceCallComplete = true;
            });

            yield return new WaitUntil(() => serviceCallComplete);

            if (response != null && response.trajectories.Length > 0)
            {
                // Execute only the first trajectory (pre-grasp)
                yield return StartCoroutine(ExecuteTrajectory(response.trajectories[0]));
            }
            else
            {
                Debug.LogError("No trajectory returned for pose movement");
            }

            m_IsExecutingTrajectory = false;
        }

        /// <summary>
        ///     Reset the pick and place state
        /// </summary>
        public void ResetPickAndPlace()
        {
            m_IsPickAndPlaceComplete = false;
            if (m_Target != null)
            {
                m_Target.transform.SetParent(null);
            }
            Debug.Log("Pick and place state reset");
        }

        /// <summary>
        ///     Close the Panda gripper (similar to Niryo)
        /// </summary>
        public void CloseGripper()
        {
            if (m_LeftGripper != null && m_RightGripper != null)
            {
                var leftDrive = m_LeftGripper.xDrive;
                var rightDrive = m_RightGripper.xDrive;

                // Close gripper - both fingers move toward each other
                leftDrive.target = 0.0f;   // Adjust for your Panda gripper
                rightDrive.target = 0.0f;  // Adjust for your Panda gripper

                m_LeftGripper.xDrive = leftDrive;
                m_RightGripper.xDrive = rightDrive;
                
                Debug.Log("Panda gripper closed");
            }
        }

        /// <summary>
        ///     Open the Panda gripper (similar to Niryo)
        /// </summary>
        public void OpenGripper()
        {
            if (m_LeftGripper != null && m_RightGripper != null)
            {
                var leftDrive = m_LeftGripper.xDrive;
                var rightDrive = m_RightGripper.xDrive;

                // Open gripper - both fingers move away from each other
                leftDrive.target = 0.04f;  // Adjust for your Panda gripper
                rightDrive.target = 0.04f; // Adjust for your Panda gripper

                m_LeftGripper.xDrive = leftDrive;
                m_RightGripper.xDrive = rightDrive;
                
                Debug.Log("Panda gripper opened");
            }
        }

        /// <summary>
        ///     Get the end-effector transform for the panda_manipulator group (includes gripper)
        /// </summary>
        Transform GetEndEffectorTransform()
        {
            // Try to find the TCP (Tool Center Point) of the panda_manipulator group
            // According to SRDF: panda_manipulator group uses panda_hand_tcp as tip_link
            var endEffector = m_PandaRobot.transform.Find("world/panda_link0/panda_link1/panda_link2/panda_link3/panda_link4/panda_link5/panda_link6/panda_link7/panda_link8/panda_hand/panda_hand_tcp");
            
            if (endEffector != null)
            {
                return endEffector;
            }
            
            // Fallback to panda_hand if panda_hand_tcp not found
            endEffector = m_PandaRobot.transform.Find("world/panda_link0/panda_link1/panda_link2/panda_link3/panda_link4/panda_link5/panda_link6/panda_link7/panda_link8/panda_hand");
            
            if (endEffector != null)
            {
                Debug.LogWarning("Using panda_hand as end-effector fallback. panda_hand_tcp not found.");
                return endEffector;
            }
            
            // Final fallback to panda_link8 (panda_arm group tip)
            endEffector = m_PandaRobot.transform.Find("world/panda_link0/panda_link1/panda_link2/panda_link3/panda_link4/panda_link5/panda_link6/panda_link7/panda_link8");
            
            if (endEffector != null)
            {
                Debug.LogWarning("Using panda_link8 as end-effector fallback. Consider using panda_hand_tcp for proper TCP.");
                return endEffector;
            }
            
            Debug.LogError("Could not find end-effector transform for Panda robot");
            return null;
        }

        /// <summary>
        ///     Test method to verify end-effector transform
        /// </summary>
        public void TestEndEffector()
        {
            Debug.Log("=== END-EFFECTOR TEST ===");
            
            var endEffector = GetEndEffectorTransform();
            if (endEffector != null)
            {
                Debug.Log($"End-effector found: {endEffector.name}");
                Debug.Log($"End-effector path: {GetTransformPath(endEffector)}");
                Debug.Log($"End-effector position: {endEffector.position}");
                Debug.Log($"End-effector rotation: {endEffector.rotation.eulerAngles}");
            }
            else
            {
                Debug.LogError("No end-effector transform found!");
            }
            
            Debug.Log("=== END END-EFFECTOR TEST ===");
        }
        
        /// <summary>
        ///     Helper method to get full transform path
        /// </summary>
        string GetTransformPath(Transform transform)
        {
            var path = transform.name;
            var parent = transform.parent;
            
            while (parent != null && parent != m_PandaRobot.transform)
            {
                path = parent.name + "/" + path;
                parent = parent.parent;
            }
            
            return path;
        }

        /// <summary>
        ///     Test method to verify MoveIt planning group configuration
        /// </summary>
        public void TestPlanningGroupConfiguration()
        {
            Debug.Log("=== PLANNING GROUP CONFIGURATION TEST ===");
            Debug.Log($"Unity Planning Group: {m_PlanningGroup}");
            Debug.Log($"Expected End-Effector for {m_PlanningGroup}: panda_hand_tcp");
            
            var endEffector = GetEndEffectorTransform();
            if (endEffector != null)
            {
                Debug.Log($"Found End-Effector: {endEffector.name}");
                Debug.Log($"Path: {GetTransformPath(endEffector)}");
                
                if (m_PlanningGroup == "panda_manipulator" && endEffector.name == "panda_hand_tcp")
                {
                    Debug.Log("✓ CONFIGURATION CORRECT: panda_manipulator group with panda_hand_tcp end-effector");
                }
                else if (m_PlanningGroup == "panda_arm" && endEffector.name == "panda_link8")
                {
                    Debug.Log("✓ CONFIGURATION CORRECT: panda_arm group with panda_link8 end-effector");
                }
                else
                {
                    Debug.LogWarning($"⚠ CONFIGURATION MISMATCH: {m_PlanningGroup} group but using {endEffector.name} end-effector");
                }
            }
            else
            {
                Debug.LogError("❌ NO END-EFFECTOR FOUND");
            }
            
            Debug.Log("=== END PLANNING GROUP CONFIGURATION TEST ===");
        }

        // Public methods for UI/testing
        public void SetTarget(GameObject target) => m_Target = target;
        public void SetTargetPlacement(GameObject target) => m_TargetPlacement = target;
        
        /// <summary>
        ///     Test method to verify button connectivity
        /// </summary>
        public void TestButtonPress()
        {
            Debug.Log("TEST: Button press detected! PandaTrajectoryPlanner is responding.");
            Debug.LogWarning("WARNING: This is a warning message to test console output.");
            Debug.LogError("ERROR: This is an error message to test console output.");
            print("PRINT: Using print() method for console output.");
        }
        
        /// <summary>
        ///     Static test method for button connectivity
        /// </summary>
        public static void StaticTestButtonPress()
        {
            Debug.Log("STATIC TEST: Static button press detected!");
            Debug.LogWarning("STATIC WARNING: This is a static warning message.");
        }
        
        /// <summary>
        ///     Simple test method with minimal dependencies
        /// </summary>
        public void SimpleTest()
        {
            Debug.Log("SIMPLE TEST: Simple test method called at " + System.DateTime.Now);
            UnityEngine.Debug.Log("UNITY DEBUG: Using full Unity namespace");
        }
        
        /// <summary>
        ///     Test method to check if all components are assigned
        /// </summary>
        public void TestSetup()
        {
            Debug.Log("=== PandaTrajectoryPlanner Setup Test ===");
            Debug.Log($"PandaRobot assigned: {m_PandaRobot != null}");
            Debug.Log($"Target assigned: {m_Target != null}");
            Debug.Log($"TargetPlacement assigned: {m_TargetPlacement != null}");
            Debug.Log($"ROS connected: {m_Ros != null}");
            Debug.Log($"Joint bodies initialized: {m_JointArticulationBodies != null}");
            Debug.Log($"Left gripper found: {m_LeftGripper != null}");
            Debug.Log($"Right gripper found: {m_RightGripper != null}");
            Debug.Log("=== End Setup Test ===");
        }
        
        /// <summary>
        ///     Test method to visualize pick approach pose
        /// </summary>
        public void TestPickPose()
        {
            if (m_Target == null)
            {
                Debug.LogError("No target assigned for pick pose test");
                return;
            }
            
            Debug.Log("=== PICK POSE TEST ===");
            
            // Calculate pick position with offset
            var pickPosition = m_Target.transform.position + m_PickPoseOffset;
            var pickPositionRelativeToRobot = pickPosition - m_PandaRobot.transform.position;
            
            Debug.Log($"Target position: {m_Target.transform.position}");
            Debug.Log($"Pick position (with {m_PickOffsetHeight}m offset): {pickPosition}");
            Debug.Log($"Pick position relative to robot: {pickPositionRelativeToRobot}");
            Debug.Log($"Pick orientation (Unity): {m_PickOrientation.eulerAngles}");
            Debug.Log($"Pick orientation (ROS): {m_PickOrientation.To<FLU>()}");
            
            // Convert to ROS coordinates
            var rosPosition = pickPositionRelativeToRobot.To<FLU>();
            var rosOrientation = m_PickOrientation.To<FLU>();
            
            Debug.Log($"ROS Pick Position: ({rosPosition.x:F3}, {rosPosition.y:F3}, {rosPosition.z:F3})");
            Debug.Log($"ROS Pick Orientation: ({rosOrientation.x:F3}, {rosOrientation.y:F3}, {rosOrientation.z:F3}, {rosOrientation.w:F3})");
            Debug.Log("=== END PICK POSE TEST ===");
        }
        
        // NOTE: End-effector reference based on MoveIt group selection:
        // - panda_arm group: tip_link = panda_link8 (flange)
        // - panda_manipulator group: tip_link = panda_hand_tcp (Tool Center Point)
        // This code assumes panda_manipulator group usage for pick-and-place operations
    }
}
