#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from tf2_ros import TransformListener, Buffer
from px4_msgs.msg import VehicleOdometry
from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped
from rtabmap_msgs.msg import OdomInfo
from std_msgs.msg import String

import numpy as np
import datetime

from transitions import Machine

class States:
    INIT = 'INIT'
    LOADING_TF = 'LOADING_TF'
    STANDBY = 'STANDBY'
    UNAVAILABLE = 'UNAVAILABLE'
    TRACKING = 'TRACKING'
    LOST = 'LOST'
    FAILSAFE = 'FAILSAFE'

class SensingManager(Node):
    def __init__(self):
        super().__init__('sensing_manager')

        # Configure QoS profile for publishing and subscribing
        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Initialize variables
        self.tf_ok = False
        self.waiting_time = 0.0
        self.dt = 0.01
        self.pose_data = []

        self.tf_world2cam = TransformStamped()
        self.odom_info = OdomInfo()

        # Init FSM
        self._init_fsm()

        # Init ROS2 interfaces
        self._init_ros_interfaces()

        # Init data storing
        self._init_file_manager()

    def _init_fsm(self):
        # FSM states and transitions
        self.states = [
            States.INIT, States.LOADING_TF, States.STANDBY, States.UNAVAILABLE, States.TRACKING,
            States.LOST, States.FAILSAFE
        ]
        self.transitions = [
            {'trigger': 'sm_load_tf', 'source': States.INIT, 'dest': States.LOADING_TF},
            {'trigger': 'sm_tf_loaded', 'source': States.LOADING_TF, 'dest': States.STANDBY},
            {'trigger': 'sm_sys_unavailable', 'source': '*', 'dest': States.UNAVAILABLE},
            {'trigger': 'sm_tracking_started', 'source': States.STANDBY, 'dest': States.TRACKING},
            {'trigger': 'sm_lost', 'source': States.TRACKING, 'dest': States.LOST},
            {'trigger': 'sm_tracking_recovered', 'source': States.LOST, 'dest': States.TRACKING},
            {'trigger': 'sm_fail', 'source': '*', 'dest': States.FAILSAFE},
            {'trigger': 'sm_reset', 'source': States.FAILSAFE, 'dest': States.INIT},
            {'trigger': 'sm_standby', 'source': '*', 'dest': States.STANDBY},
        ]
        self.machine = Machine(model=self,
                               states=self.states,
                               transitions=self.transitions,
                               initial=States.INIT,
                               after_state_change='print_sensing_state',
                               ignore_invalid_triggers=True)

    def _init_ros_interfaces(self):
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.vio_publisher = self.create_publisher(
            VehicleOdometry,
            '/fmu/in/vehicle_visual_odometry',
            self.qos_profile
        )
        self.sensing_state_publisher = self.create_publisher(
            String, 
            'palmbee/sensing_state',
            self.qos_profile
        )

        self.odom_info_subscriber = self.create_subscription(
            OdomInfo,
            '/rtabmap/odom_info',
            self.odom_info_callback,
            self.qos_profile
        )

        self.published = False

        self.timer = self.create_timer(self.dt, self.timer_callback)

    def _init_file_manager(self):
        # Create unique file name for pose data
        now = datetime.datetime.now()
        self.pose_filename = f"pose_{now.strftime('%Y%m%d_%H%M%S')}.txt"
        self.pose_file = open(self.pose_filename, 'a')

    def print_sensing_state(self):
        self.get_logger().info(f'Current state: {self.state}')

    def publish_sensing_state(self):
        msg = String()
        msg.data = self.state
        self.sensing_state_publisher.publish(msg)
    
    def publish_visual_odometry(self):
        self.get_transform()

        msg = VehicleOdometry()

        # Timestamp (convert from ROS Time)
        now = self.get_clock().now().to_msg()
        msg.timestamp = self.get_clock().now().nanoseconds // 1000  # px4 expects time in microseconds

        # Position
        msg.position = [
            self.tf_world2cam.transform.translation.x,
            self.tf_world2cam.transform.translation.y,
            self.tf_world2cam.transform.translation.z
        ]

        # Orientation
        msg.q = [
            self.tf_world2cam.transform.rotation.w,
            self.tf_world2cam.transform.rotation.x,
            self.tf_world2cam.transform.rotation.y,
            self.tf_world2cam.transform.rotation.z
        ]

        # Set velocity and covariance to unknown
        msg.velocity_frame = VehicleOdometry.VELOCITY_FRAME_UNKNOWN
        msg.pose_frame = VehicleOdometry.POSE_FRAME_FRD
        # msg.q_offset = [1.0, 0.0, 0.0, 0.0]
        msg.position_variance = [1e-3, 1e-3, 1e-3]
        msg.orientation_variance = [1e-3, 1e-3, 1e-3]
        msg.velocity_variance = [1e2, 1e2, 1e2]  # very high uncertainty

        self.pose_data = [msg.timestamp,
                          msg.position[0], msg.position[1], msg.position[2],
                          msg.q[1], msg.q[2], msg.q[3], msg.q[0]]

        self.vio_publisher.publish(msg)

    def get_transform(self):
        try:
            self.tf_world2cam = self.tf_buffer.lookup_transform(
                'world', 
                'zed_camera_link',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            self.tf_ok = True
        except Exception as e:
            self.tf_ok = False
            self.get_logger().warn(f'Looking up TF: {e}')

    def odom_info_callback(self, odom_info):
        self.odom_info = odom_info

    def timer_callback(self):
        # State machine logic
        if self.state == States.INIT:
            self.get_logger().info("Loading TF...")
            self.sm_load_tf()
        elif self.state == States.LOADING_TF:
            self.get_transform()
            if self.tf_ok:
                self.get_logger().info("TF loaded successfully.")
                self.sm_tf_loaded()
            else:
                self.get_logger().warn("TF not loaded yet, retrying...")
        elif self.state == States.STANDBY:
            if not self.odom_info.lost:
                self.get_logger().info("Tracking started...")
                self.sm_tracking_started()
            else:
                self.get_logger().warn("Waiting for tracking to start...")
        elif self.state == States.TRACKING:
            if self.odom_info:
                if self.odom_info.lost:
                    self.get_logger().info("Tracking lost...")
                    self.sm_lost()
                else:
                    self.publish_visual_odometry()
            else:
                self.get_logger().warn("Waiting for odometry info...")
                self.waiting_time += self.dt
                if self.waiting_time >= 20.0:
                    self.waiting_time = 0.0
                    self.get_logger().warn("Odometry info unavailable, tracking lost...")
                    self.sm_lost()
        elif self.state == States.LOST:
            if self.odom_info:
                if not self.odom_info.lost:
                    self.get_logger().info("Tracking recovered...")
                    self.sm_tracking_recovered()
                else:
                    self.get_logger().warn("Tracking lost, trying to recover...")
                    self.waiting_time += self.dt
                    if self.waiting_time >= 60.0:
                        self.waiting_time = 0.0
                        self.get_logger().warn("Failed to recover tracking, going to failsafe...")
                        self.sm_fail()
            else:
                self.sm_fail()
        elif self.state == States.FAILSAFE:
            self.get_logger().error("In failsafe mode, reset the system!")
            self.waiting_time += self.dt
            if self.waiting_time >= 60.0:
                self.waiting_time = 0.0
                return
        else:
            self.get_logger().error(f"Unknown state: {self.state}")
        
        # Publish sensing state
        self.publish_sensing_state()

        # Save pose_data to file if available
        if self.pose_data:
            line = ' '.join(str(x) for x in self.pose_data)
            self.pose_file.write(line + '\n')
            self.pose_file.flush()

    def __del__(self):
        try:
            self.pose_file.close()
        except Exception:
            pass

def main(args=None):
    rclpy.init(args=args)
    sm = SensingManager()
    rclpy.spin(sm)
    sm.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()