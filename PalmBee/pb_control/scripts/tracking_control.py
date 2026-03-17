#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleGlobalPosition, VehicleLocalPosition, VehicleStatus, VehicleTrajectoryWaypoint
from geographiclib.geodesic import Geodesic
import math

class TrackingControl(Node):
    """Node for controlling a vehicle in offboard mode."""

    def __init__(self) -> None:
        super().__init__('tracking_control')

        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.ns = ''
        self.target_system_id = 1

        # Create publishers
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, f'{self.ns}/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, f'{self.ns}/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, f'{self.ns}/fmu/in/vehicle_command', qos_profile)
        self.velocity_setpoint_publisher = self.create_publisher(
            VehicleTrajectoryWaypoint, f'{self.ns}/fmu/in/vehicle_trajectory_waypoint', qos_profile)

        # Create subscribers
        self.vehicle_global_position_subscriber = self.create_subscription(
            VehicleGlobalPosition, f'{self.ns}/fmu/out/vehicle_global_position', self.vehicle_global_position_callback, qos_profile)
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, f'{self.ns}/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, qos_profile)
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, f'{self.ns}/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)

        # Initialize variables
        self.offboard_setpoint_counter = 0
        self.vehicle_global_position = VehicleGlobalPosition()
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.takeoff_height = -2.0
        self.previous_error = [0.0, 0.0, 0.0]
        self.home_position = [-6.886367753433963, 107.60964063527699, 0.16148334741592407]  # Store home position for NED conversion

        self.kp = 1.8  # Proportional gain for the position controller
        self.kd = 0.6  # Derivative gain for the position controller
        self.kp_z = 0.5
        self.kd_z = 0.01

        # Waypoints
        self.waypoints = [
            [0.0, 0.0, self.takeoff_height],
            # [0.0, 1.0, self.takeoff_height],
            # [1.0, 1.0, self.takeoff_height],
            # [1.0, 0.0, self.takeoff_height],
            # [0.0, 0.0, self.takeoff_height]
        ]
        self.current_waypoint_index = 0
        self.waypoint_timer = 0

        self.dt = 0.05

        # Create a timer to publish control commands
        self.timer = self.create_timer(self.dt, self.timer_callback)

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
        """Calculate and publish position setpoint using a simple P controller (using local position)."""
        current_position = self.vehicle_local_position
        position_setpoint = TrajectorySetpoint()

        # Use current local position (NED)
        current_ned = [current_position.x, current_position.y, current_position.z]

        # Calculate position setpoint using P control
        position_setpoint.position = [
            current_ned[0] + self.kp * (desired_position[0] - current_ned[0]),
            current_ned[1] + self.kp * (desired_position[1] - current_ned[1]),
            current_ned[2] + self.kp * (desired_position[2] - current_ned[2])
        ]

        # Keep the yaw constant
        position_setpoint.yaw = float('nan')  # (0 degree)

        position_setpoint.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(position_setpoint)
        self.get_logger().info(f"Publishing position setpoints {position_setpoint.position} with yaw {position_setpoint.yaw}")

    def publish_position_setpoint_with_pd_controller(self, desired_position):
        """Calculate and publish position setpoint using a simple PD controller (using local position and estimated velocity)."""
        current_position = self.vehicle_local_position
        position_setpoint = TrajectorySetpoint()

        # Use current local position (NED)
        current_ned = [current_position.x, current_position.y, current_position.z]

        # Calculate error
        error = [
            desired_position[0] - current_ned[0],
            desired_position[1] - current_ned[1],
            desired_position[2] - current_ned[2]
        ]

        # Estimate velocity using difference with previous position and sampling time
        # Assume timer period is 0.1s as set in self.create_timer(0.1, ...)
        dt = self.dt
        if not hasattr(self, 'prev_ned'):
            self.prev_ned = current_ned
        estimated_vel = [
            (current_ned[0] - self.prev_ned[0]) / dt,
            (current_ned[1] - self.prev_ned[1]) / dt,
            (current_ned[2] - self.prev_ned[2]) / dt
        ]
        self.prev_ned = current_ned

        # Derivative term is negative estimated velocity
        d_error = [
            -estimated_vel[0],
            -estimated_vel[1],
            -estimated_vel[2]
        ]

        # PD control law
        position_setpoint.position = [
            current_ned[0] + self.kp * error[0] + self.kd * d_error[0],
            current_ned[1] + self.kp * error[1] + self.kd * d_error[1],
            current_ned[2] + self.kp_z * error[2] + self.kd_z * d_error[2]
        ]

        # Keep the yaw constant
        position_setpoint.yaw = float('nan') # (90 degree)

        position_setpoint.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(position_setpoint)
        self.get_logger().info(
            f"Publishing PD position setpoints {position_setpoint.position} with yaw {position_setpoint.yaw}"
        )

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

    def timer_callback(self) -> None:
        """Callback function for the timer."""
        self.publish_offboard_control_heartbeat_signal()

        # Log current position coordinates
        self.get_logger().info(f"Home frame (LLA): {self.vehicle_local_position.ref_lat}, {self.vehicle_local_position.ref_lon}, {self.vehicle_local_position.ref_alt}")
        self.get_logger().info(f"Current global position: lat={self.vehicle_global_position.lat}, lon={self.vehicle_global_position.lon}, alt={self.vehicle_global_position.alt}")
        self.get_logger().info(f"Current local position: x={self.vehicle_local_position.x}, y={self.vehicle_local_position.y}, z={self.vehicle_local_position.z}")

        if self.offboard_setpoint_counter == 30:
            self.engage_offboard_mode()
            self.arm()

        if self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            # Update waypoint timer
            self.waypoint_timer += 0.1
            if self.waypoint_timer >= 10.0:
                self.current_waypoint_index = (self.current_waypoint_index + 1) % len(self.waypoints)
                self.waypoint_timer = 0

            # Publish position setpoint for the current waypoint
            # self.publish_position_setpoint_with_p_controller(self.waypoints[self.current_waypoint_index])
            self.publish_position_setpoint_with_pd_controller(self.waypoints[self.current_waypoint_index])

        if self.offboard_setpoint_counter < 31:
            self.offboard_setpoint_counter += 1


def main(args=None) -> None:
    print('Starting offboard control node...')
    rclpy.init(args=args)
    offboard_control = TrackingControl()
    rclpy.spin(offboard_control)
    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)