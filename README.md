# RT1_Second-Assignment
## Safe Robot Control in Gazebo using ROS 2

---

## 1. Project Overview
This project implements a **safe control architecture for a mobile robot** simulated in Gazebo, developed using **ROS 2**.
The system allows a user to manually control the robot while ensuring safety by continuously monitoring LiDAR data and overriding unsafe commands.

The architecture is modular and follows ROS best practices, using:
- multiple ROS 2 nodes,
- custom messages,
- ROS services,
- topic separation,
- and Gazebo–ROS bridges.

---

## 2. System Architecture
The system is composed of **two main ROS 2 nodes**:
1. **Manual Control Node**
2. **Safety Node**

The interaction between nodes is shown conceptually below:

```
User Input
   ↓
manual_control_node
   ↓  (/user_cmd_vel)
safety_node
   ↓  (/safe_cmd_vel)
Gazebo (via ros_gz_bridge)
```

The robot is simulated in Gazebo and equipped with a **LiDAR sensor** publishing laser scans on `/scan`.

---

## 3. ROS 2 Nodes

### 3.1 Manual Control Node
**Purpose**  
Allows the user to drive the robot by providing linear and angular velocity commands.

**Published Topic**
- `/user_cmd_vel` (`geometry_msgs/msg/Twist`)

**Behavior**
- The user inputs linear and angular velocities from the terminal.
- Each command is **published continuously for 1 second**, then a STOP command is sent.
- The node does **not** perform any obstacle detection or safety checks.
- Safety is entirely delegated to the Safety Node, ensuring clear separation of concerns.

---

### 3.2 Safety Node
**Purpose**  
Ensures safe robot motion by filtering user commands based on obstacle proximity.

**Subscribed Topics**
- `/scan` (`sensor_msgs/msg/LaserScan`)
- `/user_cmd_vel` (`geometry_msgs/msg/Twist`)

**Published Topics**
- `/cmd_vel` (`geometry_msgs/msg/Twist`)
- `/obstacle_info` (`rt2_interfaces/msg/ObstacleInfo`)

**Logic**
- The node continuously computes the **minimum distance** from laser scan data.
- If the robot is closer than a configurable threshold to an obstacle:
  - forward motion is overridden,
  - a **backward motion with a small rotation** is commanded to return to a safe area.
- Otherwise, the user command is forwarded unchanged.

This design guarantees that **unsafe commands never reach the robot**.

---

## 4. Custom Message

### `/obstacle_info`
A custom message is published to summarize the current safety status.

**Fields**
- `closest_distance` — distance to the nearest obstacle
- `direction` — relative direction of the obstacle (`left`, `front`, `right`)
- `threshold` — current safety threshold

**Topic**
- `/obstacle_info`

**Notes**
- The direction is computed by dividing the laser scan into three sectors (left / front / right) and selecting the sector with the minimum valid distance.
- This topic is useful for debugging, monitoring, and visualization.

---

## 5. ROS Services

### 5.1 Set Safety Threshold
**Service**
- `/set_threshold` (`rt2_interfaces/srv/SetThreshold`)

**Purpose**
Allows the user to change the safety distance threshold at runtime.

**Example**
```bash
ros2 service call /set_threshold rt2_interfaces/srv/SetThreshold "{threshold: 0.4}"

---
## 5.2 Get Average Velocity

**Service**:

- `/get_avg_vel` (`rt2_interfaces/srv/
GetAvgVel`)

**Purpose**:

Returns the average linear and angular velocity over the most recent 5 safe commands, as required by the assignment.

**Returned Values**:

- average linear velocity
- average angular velocity
- window size (≤ 5)
  
**Example**:
```bash
ros2 service call /get_avg_vel rt2_interfaces/srv/GetAvgVel "{}"
```
---

# 6. Gazebo Integration

The simulation uses the provided `bme_gazebo_sensors` package.

**Features**:

- Gazebo world with obstacles
- Differential-drive mobile robot
- GPU-based LiDAR sensor
- ROS-Gazebo bridge (`ros_gz_bridge`)
 
**Bridged Topics**:

- `scan`
- `/cmd_vel`
- `/odom`
- `/joint_states`
- `/clock`
  
**Safety Integration Detail**

The Gazebo bridge subscribes to `/safe_cmd_vel`, ensuring that:
- **Only filtered commands reach the robot**
-  Unsafe user inputs are never executed in simulation
---
# 7. Launch File

The launch file:
	-	starts Gazebo,
	-	spawns the robot,
	-	launches RViz,
	-	starts the ROS–Gazebo bridge,
	-	and publishes the robot state.

**Launch file**:

`spawn_robot.launch.py`

**Example execution**:

`ros2 launch bme_gazebo_sensors spawn_robot.launch.py`

---
# 8. How to Run the Project
## 1. Build the workspace
```bash
cd ~/ros2_ws
colcon build --merge-install
source install/setup.bash
```

## 2. Launch Gazebo simulation
`ros2 launch bme_gazebo_sensors spawn_robot.launch.py`

## 3. Run Safety Node
`ros2 run rt2_assignment safety_node`

## 4. Run Manual Control Node
`ros2 run rt2_assignment manual_control`

---
# 9. Testing and Validation
The system was validated by:

- driving the robot toward obstacles,
- verifying that unsafe forward motion is overridden below the threshold,
- observing backward motion and rotation to escape obstacles,
- dynamically changing the threshold via service calls,
- monitoring `/obstacle_info` messages,
- and verifying correct velocity filtering and service responses.
  
  ---
  
  # 10. Conclusion

This project demonstrates a **robust and modular safety control architecture** for a mobile robot in ROS 2.
The design cleanly separates user control from safety enforcement and fully satisfies all requirements of the Research Track I Assignment 2.
