#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, VehicleCommand, VehicleStatus

class OffboardLanding(Node):
    def __init__(self):
        super().__init__('offboard_landing')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.offboard_control_mode_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)

        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        self.vehicle_status_sub = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.status_callback, qos_profile)

        self.setpoint_counter = 0
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.landed = False

    def status_callback(self, msg):
        if msg.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LAND:
            self.get_logger().info("Landing initiated...")
        elif msg.nav_state == VehicleStatus.NAVIGATION_STATE_MANUAL and self.landed:
            self.get_logger().info("Landed successfully. Disarming...")
            self.disarm()
            self.destroy_node()

    def timer_callback(self):
        self.publish_heartbeat()

        if self.setpoint_counter == 10:
            self.engage_offboard_mode()
            self.get_logger().info("Sending landing command...")
            self.land()
            self.landed = True

        if self.setpoint_counter < 11:
            self.setpoint_counter += 1

    def publish_heartbeat(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        self.offboard_control_mode_pub.publish(msg)

    def engage_offboard_mode(self):
        self.publish_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)

    def land(self):
        self.publish_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)

    def disarm(self):
        self.publish_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)

    def publish_command(self, command, **params):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get('param1', 0.0)
        msg.param2 = params.get('param2', 0.0)
        msg.param3 = params.get('param3', 0.0)
        msg.param4 = params.get('param4', 0.0)
        msg.param5 = params.get('param5', 0.0)
        msg.param6 = params.get('param6', 0.0)
        msg.param7 = params.get('param7', 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        self.vehicle_command_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = OffboardLanding()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)