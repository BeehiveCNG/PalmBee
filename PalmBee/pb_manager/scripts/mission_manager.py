# Python implementation of the mission_manager FSM using the `transitions` library with ROS 2 integration

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from transitions.extensions import HierarchicalMachine as Machine

class MissionManager(Node):
    def __init__(self):
        super().__init__('mission_manager_node')

        # FSM setup
        self.states = [
            'INIT',
            'STANDBY',
            'UNAVAILABLE',
            'GLOBAL_MOTION',
            'LOCAL_MOTION',
            'DONE',
            'ERROR',
            'ABORT'
        ]

        self.transitions = [
            {'trigger': 'initialize', 'source': 'INIT', 'dest': 'STANDBY'},
            {'trigger': 'fail_check', 'source': 'STANDBY', 'dest': 'UNAVAILABLE'},
            {'trigger': 'start_global', 'source': 'STANDBY', 'dest': 'GLOBAL_MOTION'},
            {'trigger': 'start_local', 'source': 'GLOBAL_MOTION', 'dest': 'LOCAL_MOTION'},
            {'trigger': 'continue_global', 'source': 'LOCAL_MOTION', 'dest': 'GLOBAL_MOTION', 'conditions': 'has_more_trees'},
            {'trigger': 'mission_complete', 'source': 'LOCAL_MOTION', 'dest': 'DONE', 'unless': 'has_more_trees'},
            {'trigger': 'error_occurred', 'source': '*', 'dest': 'ERROR'},
            {'trigger': 'abort_mission', 'source': '*', 'dest': 'ABORT'}
        ]

        self.machine = Machine(model=self, states=self.states, transitions=self.transitions, initial='INIT')

        # ROS 2 publishers and subscribers
        self.cmd_pub = self.create_publisher(String, '/motion_command', 10)
        self.status_sub = self.create_subscription(String, '/system_status', self.status_callback, 10)
        self.timer = self.create_timer(1.0, self.publish_command)

    def is_ready(self):
        return True

    def has_more_trees(self):
        return True

    def publish_command(self):
        if self.state == 'GLOBAL_MOTION':
            msg = String()
            msg.data = 'Proceeding with global motion.'
            self.cmd_pub.publish(msg)
            self.get_logger().info(f'Published: {msg.data}')

    def status_callback(self, msg):
        self.get_logger().info(f'Received: {msg.data}')
        if msg.data == 'ready' and self.state == 'INIT':
            self.initialize()
        elif msg.data == 'global_start' and self.state == 'STANDBY':
            self.start_global()
        elif msg.data == 'local_start' and self.state == 'GLOBAL_MOTION':
            self.start_local()
        elif msg.data == 'done' and self.state == 'LOCAL_MOTION':
            self.mission_complete()


def main(args=None):
    rclpy.init(args=args)
    mm = MissionManager()
    rclpy.spin(mm)
    mm.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()