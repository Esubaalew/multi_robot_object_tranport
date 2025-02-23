# Multi-Robot Object Transportation and Coordination with ROS 2

## ⚠ WARNING

To load the meshes safely, follow these steps:

1. Navigate to your workspace folder.
2. Run the following command:

```bash
export GAZEBO_MODEL_PATH=$(pwd)/src/robot_description/sdf:$GAZEBO_MODEL_PATH
```


## Objective

This project aims to demonstrate cooperative object transportation using two small, differentially driven robots in a simplified scenario. The robots will collaboratively push a lightweight object to a designated goal location, incorporating sensation (e.g., object and obstacle detection) and actuation (e.g., path planning and movement).

## Key Features

1. **Sensation and Object Detection**:
   Each robot will use proximity sensors (e.g., ultrasonic, infrared) for object and obstacle detection. This will enable them to maintain an appropriate distance from the object and avoid collisions.

2. **Decentralized Control**:
   Robots will operate independently, relying on local sensor data and basic communication to achieve the shared task. This avoids the complexities of a central control system.

3. **Proximity-Based Movement**:
   The robots will use proximity data to align themselves and collaboratively push the object. A proportional control algorithm will ensure smooth and coordinated movement.

4. **Basic Communication Using ROS 2**:
   Robots will exchange essential data, such as their current distance to the object and movement intentions, via ROS 2 topics. This facilitates coordination without requiring complex message types.

5. **Obstacle Avoidance and Path Planning**:
   A simplified path-planning algorithm will allow the robots to navigate the environment while avoiding obstacles.

6. **Visual Feedback in RViz**:
   The system's performance will be monitored using RViz to visualize the object's position and the robots' movements. This aids in debugging and performance evaluation.

## Implementation Plan

1. **Robot Setup**:
   Prepare two differentially driven robots equipped with ROS 2. Install proximity sensors for object detection and basic odometry for movement feedback.

2. **ROS 2 Communication**:
   Establish a ROS 2 network between the robots and a central workstation. Define simple message types for sharing proximity data and movement intentions.

3. **Control and Coordination**:

   - Implement a control algorithm that adjusts each robot's velocity based on proximity data.
   - Use shared data to coordinate pushing directions, ensuring balanced force distribution.

4. **Path Planning and Obstacle Avoidance**:
   Incorporate a basic algorithm for navigating toward the goal while avoiding obstacles in the environment.

5. **Goal Designation**:
   Define a method to specify the goal location for the object, such as a predefined coordinate or user-designated point in RViz.

6. **Testing and Refinement**:
   Test the system with a lightweight object in a controlled environment. Fine-tune control and path-planning parameters to achieve smooth and efficient object transportation.
```
