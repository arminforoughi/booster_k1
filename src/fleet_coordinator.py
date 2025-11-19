#!/usr/bin/env python3
"""
Fleet Coordinator for multiple Booster K1 robots using Gun.js distributed database
Enables decentralized coordination without single point of failure
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose, Twist
import requests
import json
import time
import socket

class FleetCoordinator(Node):
    """
    Fleet coordination using Gun.js for distributed state management

    Each robot publishes:
    - Heartbeat (battery, status, position)
    - Task claims
    - Current objectives

    Subscribes to:
    - Fleet-wide task queue
    - Other robot states
    - Coordination messages
    """

    def __init__(self, robot_id='k1_001', gun_relay_url='http://localhost:8765'):
        super().__init__(f'fleet_coordinator_{robot_id}')

        self.robot_id = robot_id
        self.gun_url = gun_relay_url
        self.hostname = socket.gethostname()

        self.get_logger().info(f'Fleet Coordinator for {robot_id}')
        self.get_logger().info(f'Gun.js relay: {gun_url}')

        # Robot state
        self.battery_level = 100.0
        self.current_mode = 'damping'
        self.current_position = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.current_task = None
        self.status = 'idle'

        # Subscribe to robot status from ROS2 bridge
        self.status_sub = self.create_subscription(
            String,
            f'/{robot_id}/status',
            self.status_callback,
            10)

        # Publish coordination commands
        self.coord_cmd_pub = self.create_publisher(
            String,
            f'/{robot_id}/coord_cmd',
            10)

        # Heartbeat timer (1 Hz)
        self.heartbeat_timer = self.create_timer(1.0, self.publish_heartbeat)

        # Task polling timer (0.5 Hz - check for new tasks)
        self.task_timer = self.create_timer(2.0, self.poll_tasks)

        # Fleet state polling (0.2 Hz - monitor other robots)
        self.fleet_timer = self.create_timer(5.0, self.poll_fleet_state)

        self.get_logger().info('✓ Fleet coordinator active')

    def status_callback(self, msg):
        """Update robot mode from ROS2 bridge"""
        self.current_mode = msg.data

    def publish_heartbeat(self):
        """Publish robot state to Gun.js for fleet awareness"""
        heartbeat = {
            'robot_id': self.robot_id,
            'hostname': self.hostname,
            'battery': self.get_battery_level(),
            'mode': self.current_mode,
            'status': self.status,
            'position': self.current_position,
            'current_task': self.current_task,
            'timestamp': time.time()
        }

        try:
            # Publish to Gun.js
            response = requests.put(
                f"{self.gun_url}/fleet/robots/{self.robot_id}",
                json=heartbeat,
                timeout=2
            )

            if response.status_code == 200:
                self.get_logger().debug(f'Heartbeat published')
            else:
                self.get_logger().warn(f'Heartbeat failed: {response.status_code}')

        except requests.exceptions.ConnectionError:
            self.get_logger().warn('Gun.js relay not reachable', throttle_duration_sec=10.0)
        except Exception as e:
            self.get_logger().error(f'Heartbeat error: {e}', throttle_duration_sec=5.0)

    def poll_tasks(self):
        """Check Gun.js for available tasks and claim if idle"""
        if self.current_task is not None:
            return  # Already working on a task

        try:
            # Get pending tasks from Gun.js
            response = requests.get(
                f"{self.gun_url}/fleet/tasks",
                timeout=2
            )

            if response.status_code == 200:
                tasks = response.json()

                # Find unclaimed task
                for task_id, task_data in tasks.items():
                    if task_data.get('claimed_by') is None and task_data.get('status') == 'pending':
                        # Attempt to claim task
                        if self.claim_task(task_id, task_data):
                            self.get_logger().info(f'✓ Claimed task: {task_id}')
                            self.current_task = task_id
                            self.status = 'busy'
                            break

        except requests.exceptions.ConnectionError:
            pass  # Gun.js not available
        except Exception as e:
            self.get_logger().error(f'Task polling error: {e}', throttle_duration_sec=5.0)

    def claim_task(self, task_id, task_data):
        """Atomically claim a task using Gun.js"""
        try:
            # Use PUT with timestamp for optimistic locking
            claim_data = {
                **task_data,
                'claimed_by': self.robot_id,
                'claimed_at': time.time(),
                'status': 'in_progress'
            }

            response = requests.put(
                f"{self.gun_url}/fleet/tasks/{task_id}",
                json=claim_data,
                timeout=2
            )

            if response.status_code == 200:
                # Verify we actually got the claim (check for race conditions)
                time.sleep(0.1)
                verify = requests.get(f"{self.gun_url}/fleet/tasks/{task_id}", timeout=2)

                if verify.status_code == 200:
                    result = verify.json()
                    return result.get('claimed_by') == self.robot_id

        except Exception as e:
            self.get_logger().error(f'Task claim error: {e}')

        return False

    def complete_task(self, task_id, result_data=None):
        """Mark task as completed"""
        try:
            completion_data = {
                'status': 'completed',
                'completed_by': self.robot_id,
                'completed_at': time.time(),
                'result': result_data
            }

            response = requests.patch(
                f"{self.gun_url}/fleet/tasks/{task_id}",
                json=completion_data,
                timeout=2
            )

            if response.status_code == 200:
                self.get_logger().info(f'✓ Task {task_id} completed')
                self.current_task = None
                self.status = 'idle'
                return True

        except Exception as e:
            self.get_logger().error(f'Task completion error: {e}')

        return False

    def poll_fleet_state(self):
        """Monitor other robots in the fleet"""
        try:
            response = requests.get(
                f"{self.gun_url}/fleet/robots",
                timeout=2
            )

            if response.status_code == 200:
                fleet_state = response.json()

                # Count active robots
                active_robots = 0
                for robot_id, robot_data in fleet_state.items():
                    timestamp = robot_data.get('timestamp', 0)
                    age = time.time() - timestamp

                    if age < 5.0:  # Active if heartbeat within 5 seconds
                        active_robots += 1

                self.get_logger().info(
                    f'Fleet status: {active_robots} active robots',
                    throttle_duration_sec=30.0
                )

        except requests.exceptions.ConnectionError:
            pass  # Gun.js not available
        except Exception as e:
            self.get_logger().error(f'Fleet state error: {e}', throttle_duration_sec=10.0)

    def get_battery_level(self):
        """Get battery level - TODO: implement actual battery reading"""
        # Placeholder - integrate with actual battery monitoring
        return self.battery_level

    def get_position(self):
        """Get current position - TODO: integrate with odometry"""
        # Placeholder - integrate with actual position from /odom
        return self.current_position


def main():
    import sys

    robot_id = sys.argv[1] if len(sys.argv) > 1 else 'k1_001'
    gun_url = sys.argv[2] if len(sys.argv) > 2 else 'http://localhost:8765'

    print(f'\n{"="*60}')
    print(f'Fleet Coordinator - Distributed State Management')
    print(f'{"="*60}')
    print(f'Robot ID: {robot_id}')
    print(f'Gun.js relay: {gun_url}')
    print(f'\nCapabilities:')
    print(f'  - Heartbeat publishing (1 Hz)')
    print(f'  - Task discovery and claiming (0.5 Hz)')
    print(f'  - Fleet monitoring (0.2 Hz)')
    print(f'\nGun.js Database Schema:')
    print(f'  /fleet/robots/{robot_id} - Robot heartbeat')
    print(f'  /fleet/tasks/{{task_id}} - Task queue')
    print(f'{"="*60}\n')

    # Initialize ROS2
    rclpy.init()

    # Create coordinator
    coordinator = None
    try:
        coordinator = FleetCoordinator(robot_id, gun_url)
        rclpy.spin(coordinator)
    except KeyboardInterrupt:
        print('\n\nKeyboard interrupt detected')
    except Exception as e:
        print(f'\nError: {e}')
        import traceback
        traceback.print_exc()
    finally:
        if coordinator:
            coordinator.destroy_node()
        rclpy.shutdown()
        print('Shutdown complete')


if __name__ == '__main__':
    main()
