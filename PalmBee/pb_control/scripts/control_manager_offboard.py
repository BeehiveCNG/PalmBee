#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import String
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleGlobalPosition, VehicleLocalPosition, VehicleStatus, VehicleTrajectoryWaypoint
from geographiclib.geodesic import Geodesic
import numpy as np

from transitions import Machine

class States:
    INIT = 'INIT'
    STANDBY = 'STANDBY'
    UNAVAILABLE = 'UNAVAILABLE'
    HOVERING = 'HOVERING'
    ARMING = 'ARMING'
    DISARMING = 'DISARMING'
    TAKING_OFF = 'TAKING_OFF'
    LANDING = 'LANDING'
    POSE_TRACKING = 'POSE_TRACKING'
    FAILSAFE = 'FAILSAFE'

class ControlManager(Node):
    """Node for controlling a vehicle in offboard mode with FSM."""

    def __init__(self) -> None:
        super().__init__('control_manager')

        # FSM states and transitions
        self._init_fsm()
        
        self.ns = ''
        self.target_system_id = 1
        self.dt = 0.1

        # Localise publisher, subscriber, and timer creation
        self._init_ros_interfaces()

        # Initialize variables
        self.offboard_setpoint_counter = 0
        self.vehicle_global_position = VehicleGlobalPosition()
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.takeoff_height = -1.5
        self.previous_error = [0.0, 0.0, 0.0]
        self.home_position = [-6.886367753433963, 107.60964063527699, 0.16148334741592407]  # Store home position for NED conversion

        # Separate gains for horizontal (x, y) and vertical (z)
        self.kp_xy = 1.5  # Proportional gain for x, y
        self.kp_z = 1.2   # Proportional gain for z
        self.kd_xy = 0.2  # Derivative gain for x, y
        self.kd_z = 0.15   # Derivative gain for z

        # Waypoints
        self.waypoints = [
            [0.0, 0.0, self.takeoff_height],
        ]
        self.last_waypoint = []

        self.current_waypoint_index = 0
        self.waypoint_timer = 0

        # Initialize pose tracking timer
        self.waiting_time = 0.0

        self.operation_counter = 0  # Counter for operations
    
    def _init_fsm(self):
        self.states = [
            States.INIT, States.STANDBY, States.UNAVAILABLE, States.HOVERING,
            States.ARMING, States.DISARMING, States.TAKING_OFF, States.LANDING,
            States.POSE_TRACKING, States.FAILSAFE
        ]
        self.transitions = [
            {'trigger': 'system_ready', 'source': States.INIT, 'dest': States.STANDBY},
            {'trigger': 'system_unavailable', 'source': '*', 'dest': States.UNAVAILABLE},
            {'trigger': 'start_hover', 'source': [States.STANDBY, States.TAKING_OFF, States.POSE_TRACKING], 'dest': States.HOVERING},
            {'trigger': 'start_arming', 'source': States.STANDBY, 'dest': States.ARMING},
            {'trigger': 'start_disarming', 'source': '*', 'dest': States.DISARMING},
            {'trigger': 'start_taking_off', 'source': States.ARMING, 'dest': States.TAKING_OFF},
            {'trigger': 'start_landing', 'source': '*', 'dest': States.LANDING},
            {'trigger': 'track_pose', 'source': [States.HOVERING, States.TAKING_OFF], 'dest': States.POSE_TRACKING},
            {'trigger': 'fail', 'source': '*', 'dest': States.FAILSAFE},
            {'trigger': 'reset', 'source': States.FAILSAFE, 'dest': States.INIT},
            {'trigger': 'standby', 'source': '*', 'dest': States.STANDBY},
        ]
        self.machine = Machine(model=self,
                               states=self.states,
                               transitions=self.transitions,
                               initial=States.INIT,
                               after_state_change='print_control_state',
                               ignore_invalid_triggers=True)
        
    def _init_ros_interfaces(self):
        # Configure QoS profile for publishing and subscribing
        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        """Initialise all ROS2 publishers, subscribers, and timer."""
        # Publishers
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode,
            f'{self.ns}/fmu/in/offboard_control_mode',
            self.qos_profile
        )
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint,
            f'{self.ns}/fmu/in/trajectory_setpoint',
            self.qos_profile
        )
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand,
            f'{self.ns}/fmu/in/vehicle_command',
            self.qos_profile
        )
        self.velocity_setpoint_publisher = self.create_publisher(
            VehicleTrajectoryWaypoint,
            f'{self.ns}/fmu/in/vehicle_trajectory_waypoint',
            self.qos_profile
        )
        self.state_publisher = self.create_publisher(
            String, 
            'palmbee/control_state',
            self.qos_profile
        )

        # Subscribers
        self.vehicle_global_position_subscriber = self.create_subscription(
            VehicleGlobalPosition,
            f'{self.ns}/fmu/out/vehicle_global_position',
            self.vehicle_global_position_callback,
            self.qos_profile
        )
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition,
            f'{self.ns}/fmu/out/vehicle_local_position',
            self.vehicle_local_position_callback,
            self.qos_profile
        )
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus,
            f'{self.ns}/fmu/out/vehicle_status',
            self.vehicle_status_callback,
            self.qos_profile
        )

        # Timer
        self.timer = self.create_timer(
            self.dt, 
            self.timer_callback
        )

    def vehicle_global_position_callback(self, vehicle_global_position):
        """Callback function for vehicle_global_position topic subscriber."""
        self.vehicle_global_position = vehicle_global_position
        if self.home_position is None:
            self.home_position = (vehicle_global_position.lat, vehicle_global_position.lon, vehicle_global_position.alt)
    
    def vehicle_local_position_callback(self, vehicle_local_position):
        """Callback function for vehicle_local_position topic subscriber."""
        self.vehicle_local_position = vehicle_local_position

    def vehicle_status_callback(self, vehicle_status):
        """Callback function for vehicle_status topic subscriber."""
        self.vehicle_status = vehicle_status

    def arm(self):
        """Send an arm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('Arm command sent')

    def disarm(self):
        """Send a disarm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info('Disarm command sent')

    def engage_offboard_mode(self):
        """Switch to offboard mode."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("Switching to offboard mode")

    def land(self):
        """Switch to land mode."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info("Switching to land mode")

    def publish_offboard_control_heartbeat_signal(self):
        """Publish the offboard control mode."""
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_position_setpoint(self, x: float, y: float, z: float):
        """Publish the trajectory setpoint."""
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = 1.57079  # (90 degree)
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)
        self.get_logger().info(f"Publishing position setpoints {[x, y, z]}")

    def publish_vehicle_command(self, command, **params) -> None:
        """Publish a vehicle command."""
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.param3 = params.get("param3", 0.0)
        msg.param4 = params.get("param4", 0.0)
        msg.param5 = params.get("param5", 0.0)
        msg.param6 = params.get("param6", 0.0)
        msg.param7 = params.get("param7", 0.0)
        msg.target_system = self.target_system_id
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

    def publish_system_state(self):
        msg = String()
        msg.data = str(self.state)
        self.state_publisher.publish(msg)

    def lla_to_ned(self, lat, lon, alt):
        """Convert LLA coordinates to NED coordinates."""
        geod = Geodesic.WGS84
        home_lat, home_lon, home_alt = self.home_position
        g = geod.Inverse(home_lat, home_lon, lat, lon)
        north = g['s12'] * (1 if g['azi1'] >= 0 else -1)
        east = g['s12'] * (1 if g['azi1'] >= 90 else -1)
        down = home_alt - alt
        return north, east, down

    def publish_position_setpoint_with_p_controller(self, desired_position):
        """P controller using vector/matrix notation, separate gains for xy and z."""
        current_position = self.vehicle_local_position
        self.position_setpoint = TrajectorySetpoint()

        # State vectors
        current_ned = np.array([current_position.x, current_position.y, current_position.z])
        desired_ned = np.array(desired_position)

        # Gain matrix (diagonal)
        Kp = np.diag([self.kp_xy, self.kp_xy, self.kp_z])

        # P control law: u = Kp * (desired - current)
        control = Kp @ (desired_ned - current_ned)
        self.position_setpoint.position = (current_ned + control).tolist()

        # Keep the yaw constant
        self.position_setpoint.yaw = 0.0

        self.position_setpoint.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(self.position_setpoint)

    def publish_position_setpoint_with_pd_controller(self, desired_position):
        """PD controller using vector/matrix notation, separate gains for xy and z."""
        current_position = self.vehicle_local_position
        self.position_setpoint = TrajectorySetpoint()

        # State vectors
        current_ned = np.array([current_position.x, current_position.y, current_position.z])
        desired_ned = np.array(desired_position)

        # Proportional gain matrix
        Kp = np.diag([self.kp_xy, self.kp_xy, self.kp_z])
        # Derivative gain matrix
        Kd = np.diag([self.kd_xy, self.kd_xy, self.kd_z])

        # Error
        error = desired_ned - current_ned

        # Estimate velocity using difference with previous position and sampling time
        dt = self.dt
        if not hasattr(self, 'prev_ned'):
            self.prev_ned = current_ned
        estimated_vel = (current_ned - self.prev_ned) / dt
        self.prev_ned = current_ned

        # Derivative term is negative estimated velocity
        d_error = -estimated_vel

        # PD control law: u = Kp * error + Kd * d_error
        control = Kp @ error + Kd @ d_error
        self.position_setpoint.position = (current_ned + control).tolist()

        # Keep the yaw constant
        self.position_setpoint.yaw = 0.0

        self.position_setpoint.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(self.position_setpoint)

    def print_control_state(self):
        self.get_logger().info(f'Current state: {self.state}')

    def timer_callback(self) -> None:
        """Callback function for the timer."""
        self.publish_offboard_control_heartbeat_signal()

        # Reset waiting_time on state change
        if not hasattr(self, 'last_state'):
            self.last_state = self.state
        if self.state != self.last_state:
            self.waiting_time = 0.0
            self.last_state = self.state

        # FSM logic
        if self.state == States.INIT:
            # PX4 arming_state: 1 = DISARMED, 2 = ARMED
            if self.vehicle_status.arming_state == self.vehicle_status.ARMING_STATE_DISARMED:
                self.system_ready()
            self.waiting_time += self.dt
            if self.waiting_time >= 5.0:
                self.start_landing()
                self.waiting_time = 0.0
        elif self.state == States.STANDBY:
            if self.operation_counter == 0:
                self.start_arming()
        elif self.state == States.ARMING:
            self.engage_offboard_mode()
            self.arm()
            if self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                self.start_taking_off()
        elif self.state == States.TAKING_OFF:
            self.publish_position_setpoint_with_pd_controller(self.waypoints[self.current_waypoint_index])
            self.waiting_time += self.dt
            if self.waiting_time >= 20.0:
                self.last_waypoint = self.waypoints[self.current_waypoint_index]
                self.waiting_time = 0.0  # reset timer
                self.start_hover()
        elif self.state == States.HOVERING:
            self.publish_position_setpoint_with_pd_controller(self.last_waypoint)
            self.waiting_time += self.dt
            if self.waiting_time >= 20.0:
                self.waiting_time = 0.0  # reset timer
                self.track_pose()
        elif self.state == States.POSE_TRACKING:
            self.publish_position_setpoint_with_pd_controller(self.waypoints[self.current_waypoint_index])
            self.waiting_time += self.dt
            if self.waiting_time >= 20.0:
                self.last_waypoint = self.waypoints[self.current_waypoint_index]
                self.waiting_time = 0.0  # reset timer
                self.start_landing()  # use the new trigger name
        elif self.state == States.LANDING:
            self.land()
            self.waiting_time += self.dt
            if self.waiting_time >= 20.0:
                self.waiting_time = 0.0  # reset timer
                self.start_disarming()  # use the new trigger name
        elif self.state == States.DISARMING:
            # Only disarm once per entry
            if not hasattr(self, 'disarm_sent') or not self.disarm_sent:
                self.disarm()
                self.disarm_sent = True
            else:
                self.operation_counter += 1
                self.standby()
                self.disarm_sent = False
        elif self.state == States.FAILSAFE:
            self.get_logger().warn("FAILSAFE state reached!")
        elif self.state == States.UNAVAILABLE:
            self.get_logger().warn("System unavailable!")
        else:
            self.get_logger().error(f"Unknown state: {self.state}")

        # Log current position coordinates (optional: reduce frequency)
        if not hasattr(self, 'log_counter'):
            self.log_counter = 0
        self.log_counter += 1
        if self.log_counter >= int(2.0 / self.dt):  # log every 1 second
            # self.get_logger().info(f"Home (LLA): {self.vehicle_local_position.ref_lat}, {self.vehicle_local_position.ref_lon}, {self.vehicle_local_position.ref_alt}")
            # self.get_logger().info(f"Global (LLA): lat={self.vehicle_global_position.lat}, lon={self.vehicle_global_position.lon}, alt={self.vehicle_global_position.alt}")
            self.get_logger().info(f"Local: x={self.vehicle_local_position.x:.3f}, y={self.vehicle_local_position.y:.3f}, z={self.vehicle_local_position.z:.3f}, heading={self.vehicle_local_position.heading:.3f}")
            if self.position_setpoint.position.all()!=0.0:
                pos = self.position_setpoint.position 
                yaw = self.position_setpoint.yaw
                self.get_logger().info(f"Pose setpoint x={pos[0]:.3f}, y={pos[1]:.3f}, z={pos[2]:.3f}, yaw={yaw:.3f}")
            self.log_counter = 0

        # Publish the current FSM state
        self.publish_system_state()

def main(args=None):
    print('Starting Control Manager Node...')
    rclpy.init(args=args)
    cm = ControlManager()
    rclpy.spin(cm)
    cm.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)