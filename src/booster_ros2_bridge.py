#!/usr/bin/env python3
"""
ROS2 Bridge for Booster K1 SDK
Provides standard ROS2 interfaces (/cmd_vel, /joint_states, /odom) for fleet integration
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from booster_robotics_sdk_python import B1LocoClient, ChannelFactory, RobotMode, GetModeResponse
import time
import sys

class BoosterROS2Bridge(Node):
    """Bridge between ROS2 topics and Booster SDK for multi-robot fleet operations"""

    def __init__(self, robot_id='k1_001', network_interface='127.0.0.1'):
        super().__init__(f'booster_bridge_{robot_id}')

        self.robot_id = robot_id
        self.get_logger().info(f'Initializing ROS2 bridge for robot: {robot_id}')

        # Initialize SDK
        try:
            ChannelFactory.Instance().Init(0, network_interface)
            self.client = B1LocoClient()
            self.client.Init()
            self.get_logger().info(f'✓ SDK initialized on {network_interface}')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize SDK: {e}')
            raise

        # Verify connection
        time.sleep(0.5)
        if not self._verify_connection():
            self.get_logger().warn('Could not verify SDK connection - proceeding anyway')

        # Subscribe to standard ROS2 cmd_vel topic
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            f'/{robot_id}/cmd_vel',
            self.cmd_vel_callback,
            10)

        # Subscribe to mode change requests
        self.mode_sub = self.create_subscription(
            String,
            f'/{robot_id}/mode_cmd',
            self.mode_callback,
            10)

        # Publishers for robot state
        self.joint_state_pub = self.create_publisher(
            JointState,
            f'/{robot_id}/joint_states',
            10)

        self.odom_pub = self.create_publisher(
            Odometry,
            f'/{robot_id}/odom',
            10)

        self.status_pub = self.create_publisher(
            String,
            f'/{robot_id}/status',
            10)

        # Publish status at 10 Hz
        self.status_timer = self.create_timer(0.1, self.publish_status)

        # Safety watchdog - stop if no cmd_vel received for 1 second
        self.last_cmd_vel_time = time.time()
        self.watchdog_timer = self.create_timer(0.1, self.watchdog_callback)

        self.get_logger().info('✓ ROS2 bridge ready')
        self.get_logger().info(f'Subscribed to: /{robot_id}/cmd_vel')
        self.get_logger().info(f'Publishing to: /{robot_id}/{{joint_states, odom, status}}')

    def _verify_connection(self):
        """Verify SDK connection to robot"""
        try:
            gm = GetModeResponse()
            res = self.client.GetMode(gm)
            if res == 0:
                mode_names = {
                    RobotMode.kDamping: "Damping",
                    RobotMode.kPrepare: "Prepare",
                    RobotMode.kWalking: "Walking",
                    RobotMode.kCustom: "Custom"
                }
                mode = mode_names.get(gm.mode, "Unknown")
                self.get_logger().info(f'✓ Connected - Current mode: {mode}')
                return True
        except Exception as e:
            self.get_logger().error(f'Connection verification failed: {e}')
        return False

    def cmd_vel_callback(self, msg):
        """Convert ROS2 Twist to SDK Move command"""
        try:
            # Update watchdog
            self.last_cmd_vel_time = time.time()

            # Send to SDK (50 Hz internal rate)
            self.client.Move(
                msg.linear.x,   # Forward/backward
                msg.linear.y,   # Left/right strafe
                msg.angular.z   # Rotation
            )

        except Exception as e:
            self.get_logger().error(f'Error in cmd_vel callback: {e}')

    def mode_callback(self, msg):
        """Handle mode change requests"""
        mode_str = msg.data.lower()
        mode_map = {
            'damping': RobotMode.kDamping,
            'prepare': RobotMode.kPrepare,
            'walking': RobotMode.kWalking,
            'custom': RobotMode.kCustom
        }

        if mode_str in mode_map:
            try:
                res = self.client.ChangeMode(mode_map[mode_str])
                if res == 0:
                    self.get_logger().info(f'Mode changed to: {mode_str}')
                else:
                    self.get_logger().error(f'Mode change failed: error {res}')
            except Exception as e:
                self.get_logger().error(f'Error changing mode: {e}')
        else:
            self.get_logger().warn(f'Unknown mode: {mode_str}')

    def watchdog_callback(self):
        """Safety watchdog - stop robot if no commands received"""
        elapsed = time.time() - self.last_cmd_vel_time

        if elapsed > 1.0:  # 1 second timeout
            # Send stop command
            try:
                self.client.Move(0.0, 0.0, 0.0)
            except:
                pass

    def publish_status(self):
        """Publish robot status"""
        try:
            # Get current mode
            gm = GetModeResponse()
            res = self.client.GetMode(gm)

            if res == 0:
                mode_names = {
                    RobotMode.kDamping: "damping",
                    RobotMode.kPrepare: "prepare",
                    RobotMode.kWalking: "walking",
                    RobotMode.kCustom: "custom"
                }
                mode_str = mode_names.get(gm.mode, "unknown")

                # Publish status
                status_msg = String()
                status_msg.data = mode_str
                self.status_pub.publish(status_msg)

        except Exception as e:
            self.get_logger().error(f'Error publishing status: {e}', throttle_duration_sec=5.0)

    def shutdown(self):
        """Clean shutdown - send stop commands"""
        self.get_logger().info('Shutting down - sending stop commands')
        for _ in range(5):
            try:
                self.client.Move(0.0, 0.0, 0.0)
                time.sleep(0.02)
            except:
                pass


def main():
    # Parse arguments
    robot_id = sys.argv[1] if len(sys.argv) > 1 else 'k1_001'
    network_interface = sys.argv[2] if len(sys.argv) > 2 else '127.0.0.1'

    print(f'\n{"="*60}')
    print(f'Booster K1 ROS2 Bridge')
    print(f'{"="*60}')
    print(f'Robot ID: {robot_id}')
    print(f'Network: {network_interface}')
    print(f'\nTopics:')
    print(f'  Subscribe: /{robot_id}/cmd_vel (geometry_msgs/Twist)')
    print(f'  Subscribe: /{robot_id}/mode_cmd (std_msgs/String)')
    print(f'  Publish:   /{robot_id}/status (std_msgs/String)')
    print(f'  Publish:   /{robot_id}/joint_states (sensor_msgs/JointState)')
    print(f'  Publish:   /{robot_id}/odom (nav_msgs/Odometry)')
    print(f'\nUsage:')
    print(f'  ros2 topic pub /{robot_id}/cmd_vel geometry_msgs/Twist "{{linear: {{x: 0.5}}}}"')
    print(f'  ros2 topic pub /{robot_id}/mode_cmd std_msgs/String "data: walking"')
    print(f'{"="*60}\n')

    # Initialize ROS2
    rclpy.init()

    # Create bridge
    bridge = None
    try:
        bridge = BoosterROS2Bridge(robot_id, network_interface)
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        print('\n\nKeyboard interrupt detected')
    except Exception as e:
        print(f'\nError: {e}')
        import traceback
        traceback.print_exc()
    finally:
        if bridge:
            bridge.shutdown()
            bridge.destroy_node()
        rclpy.shutdown()
        print('Shutdown complete')


if __name__ == '__main__':
    main()
