# Rocky Times Challenge - Search, Map, & Analyze
## Report

**Name:** Sathvik Merugu  
**ASU ID:** 1235373752 

---

## Introduction

This project implements an autonomous drone system in ROS2 designed to explore and analyze cylindrical geological structures in a terrain like simulated environment. Leveraging PX4 SITL, MAVROS, and simulated RGB-D sensing, the system demonstrates autonomous takeoff, position control, and groundwork for object-based landing. This work represents a core step toward full autonomy in space robotics missions requiring intelligent terrain interaction and energy-efficient control.

---

## System Overview

### Software and Simulation Stack
- **Operating System:** Ubuntu 24.04  
- **Middleware:** ROS2 Jazzy  
- **Simulator:** PX4 SITL with `gz_x500_depth_mono` drone  
- **Sensor Model:** RGB-D (depth + color camera)  
- **Perception:** RTAB-Map SLAM (to be integrated)  
- **Control Layer:** MAVROS for OFFBOARD control and arming  

The simulation executed with a **real-time factor of ~8.49**, reflecting limited compute resources. Despite this, the drone successfully performed all required commands, including transitioning to OFFBOARD mode, arming, and maintaining stable hover. This confirms correct system integration and controller performance under constrained conditions.

---

## Methodology

### Autonomous Takeoff Flow
- A continuous stream of 100 position setpoints is first published to meet PX4’s OFFBOARD requirement  
- The drone is then switched to `OFFBOARD` mode via MAVROS  
- Once in OFFBOARD, the drone is armed using a service call  
- The drone autonomously ascends to 2.5 meters and maintains position

The setpoint remains constant (x=0, y=0, z=2.5), allowing for a smooth vertical lift. The node uses a 10 Hz timer to publish commands and monitor state changes.

---

## Implementation Details

The `takeoff_node.py` script includes:
- State monitoring (`/mavros/state`)
- Setpoint publication (`/mavros/setpoint_position/local`)
- Mode switching and arming services (`/mavros/set_mode`, `/mavros/cmd/arming`)
- Asynchronous callbacks for safe transition handling

This modular setup allows for later extension to include dynamic navigation and object-based positioning logic.

---

## Results and Observations

- Successful transition to OFFBOARD mode  
- Autonomous arming and takeoff  
- Stable hover at 2.5 meters  
- Verified behavior through simulation and visual confirmation  


*Note:* The full landing on the cylinder was **partially developed** but not completed. However, the takeoff and hover stages were successfully validated and demonstrated in the provided video.



## Demonstration Video




https://github.com/user-attachments/assets/d3261b74-8c0d-4c39-9628-03bbce5cee72




---

## Conclusion

Despite limited real-time simulation speed, the system successfully demonstrates autonomous drone control including arming, OFFBOARD mode switching, and stable hovering. While the final cylinder landing module was not fully completed, the foundation for object-aware navigation has been established. The current implementation confirms reliable low-level control and prepares the system for full terrain-based landing in future extensions.

## takeoff_node.py (Full Code)

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from geometry_msgs.msg import PoseStamped

class TakeoffNode(Node):
    def __init__(self):
        super().__init__('takeoff_node')
        self.current_state = State()
        self.connected = False

        self.setpoint = PoseStamped()
        self.setpoint.pose.position.x = 0.0
        self.setpoint.pose.position.y = 0.0
        self.setpoint.pose.position.z = 2.5  # Takeoff height

        self.state_sub = self.create_subscription(State, '/mavros/state', self.state_callback, 10)
        self.setpoint_pub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)

        self.arm_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.mode_client = self.create_client(SetMode, '/mavros/set_mode')

        self.setpoint_count = 0
        self.mode_sent = False
        self.armed_sent = False

        self.timer = self.create_timer(0.1, self.timer_callback)

    def state_callback(self, msg):
        self.current_state = msg
        self.connected = msg.connected

    def timer_callback(self):
        if not self.connected:
            self.get_logger().info("Waiting for FCU connection...")
            return

        self.setpoint_pub.publish(self.setpoint)

        if self.setpoint_count < 100:
            self.setpoint_count += 1
            return

        if not self.mode_sent and self.current_state.mode != 'OFFBOARD':
            mode_request = SetMode.Request()
            mode_request.custom_mode = 'OFFBOARD'
            future = self.mode_client.call_async(mode_request)
            future.add_done_callback(self.mode_response_callback)
            self.mode_sent = True

        if not self.armed_sent and self.current_state.mode == 'OFFBOARD' and not self.current_state.armed:
            arm_request = CommandBool.Request()
            arm_request.value = True
            future = self.arm_client.call_async(arm_request)
            future.add_done_callback(self.arm_response_callback)
            self.armed_sent = True

    def mode_response_callback(self, future):
        result = future.result()
        if result and result.mode_sent:
            self.get_logger().info("OFFBOARD mode activated.")
        else:
            self.get_logger().warn("OFFBOARD mode activation failed.")

    def arm_response_callback(self, future):
        result = future.result()
        if result and result.success:
            self.get_logger().info("Drone armed successfully.")
        else:
            self.get_logger().warn("Drone arming failed.")


def main(args=None):
    rclpy.init(args=args)
    node = TakeoffNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

---

