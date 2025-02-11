# Automated Surface Damage Detection with an Embodied Intelligence Robotic Platform

## Overview

This project focuses on automated surface damage detection using an embodied intelligence robotic platform. The system leverages a YOLO-based object detection model and pose adjustment mechanisms to enhance defect inspection capabilities. The robotic platform is managed through a ROS-based framework, with multiple nodes working together to achieve precise and efficient inspection.

## Components

### Main Node (Inspection)

The main node serves as the core controller of the robot, responsible for directing and coordinating all subsystems. Its primary functions include:

- Controlling the robot's movement
- Managing damage detection
- Adjusting position for better inspection
- Initializing and supervising all subsystems

The `pose_adjustment.py` script is located in the `pose_adjustment/include` directory.

### Path Publisher Node

The path_publisher node is responsible for tracking and publishing the robot's movement path. It retrieves the robot's position from the odom_node (odometry) and generates a list of followed waypoints, which are then published for visualization on a map.

### YOLO Model

The defect detection system uses a YOLO-based object detection model.

- `.engine` file: A TensorRT-optimized version of the model for efficient inference on the Jetson platform.
- `.pt` file: The original PyTorch model used for training and potential further fine-tuning.

The `.engine` and `.pt` models can be found in the `pose_adjustment/model` directory.

## ROS Launch Files

Launch files in ROS are XML-based scripts used to start multiple nodes simultaneously. They simplify the process of initializing complex robotic systems by automating the execution of required components. In this project, the launch files are responsible for:

- Initializing the main node and all supporting nodes
- Loading and configuring the YOLO model for defect detection
- Starting the odometry system, LiDAR, camera, IMU, robot controller, and path tracking
- Ensuring proper communication between subsystems

These launch files streamline the deployment process, making it easier to start the inspection system with minimal manual intervention.

## Usage

To launch the system, run the appropriate launch file using:

```bash
roslaunch <package_name> pose_adjustment.launch
```

## Future Enhancements

In future work, the framework’s damage detection capabilities could be enhanced by integrating cloud-based computational models for advanced segmentation, classification, and structural integrity assessments, while deep reinforcement learning (DRL) could refine autonomy by enabling adaptive navigation and optimizing inspection workflows; additionally, embedding 3D spatial mapping would correlate detected damages with structural geometry, aiding engineers in prioritizing maintenance interventions.

## Contact

For more information, please contact: kxa385@student.bham.ac.uk