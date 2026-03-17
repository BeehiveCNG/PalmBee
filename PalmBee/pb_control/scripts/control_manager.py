#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from mavros_msgs.srv import WaypointPush, WaypointClear
from mavros_msgs.msg import Waypoint
from sensor_msgs.msg import NavSatFix


class EKFOriginMissionUploader(Node):

    def __init__(self):
        super().__init__('ekf_origin_mission_uploader')

        self.origin_received = False

        # Subscribe to EKF origin
        self.origin_sub = self.create_subscription(
            NavSatFix,
            '/mavros/global_position/gp_origin',
            self.origin_cb,
            10
        )

        # Mission services
        self.clear_cli = self.create_client(
            WaypointClear,
            '/mavros/mission/clear'
        )
        self.push_cli = self.create_client(
            WaypointPush,
            '/mavros/mission/push'
        )

        self.get_logger().info('Waiting for EKF origin...')

    def origin_cb(self, msg: NavSatFix):
        if self.origin_received:
            return

        self.origin_received = True
        self.get_logger().info(
            f"EKF origin received (lat={msg.latitude}, lon={msg.longitude})"
        )

        self.upload_mission()

    def upload_mission(self):
        while not self.clear_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for mission clear service...')

        while not self.push_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for mission push service...')

        takeoff_alt = 1.5    # meters
        move_dist = 2.0      # meters
        loiter_time = 2.0    # seconds

        waypoints = []

        # 0️⃣ TAKEOFF
        wp = Waypoint()
        wp.frame = Waypoint.FRAME_LOCAL_NED
        wp.command = Waypoint.NAV_TAKEOFF
        wp.is_current = True
        wp.autocontinue = True
        wp.param7 = takeoff_alt
        waypoints.append(wp)

        # 1️⃣ MOVE 2m FROM ORIGIN
        wp = Waypoint()
        wp.frame = Waypoint.FRAME_LOCAL_NED
        wp.command = Waypoint.NAV_WAYPOINT
        wp.is_current = False
        wp.autocontinue = True
        wp.x_lat = move_dist     # North
        wp.y_long = 0.0          # East
        wp.z_alt = -takeoff_alt # Up
        waypoints.append(wp)

        # 2️⃣ HOVER / LOITER
        wp = Waypoint()
        wp.frame = Waypoint.FRAME_LOCAL_NED
        wp.command = Waypoint.NAV_LOITER_TIME
        wp.is_current = False
        wp.autocontinue = True
        wp.param1 = loiter_time
        wp.x_lat = move_dist
        wp.y_long = 0.0
        wp.z_alt = -takeoff_alt
        waypoints.append(wp)

        # 3️⃣ LAND
        wp = Waypoint()
        wp.frame = Waypoint.FRAME_LOCAL_NED
        wp.command = Waypoint.NAV_LAND
        wp.is_current = False
        wp.autocontinue = True
        waypoints.append(wp)

        # Clear mission
        self.clear_cli.call_async(WaypointClear.Request())

        # Upload mission
        req = WaypointPush.Request()
        req.waypoints = waypoints

        future = self.push_cli.call_async(req)
        future.add_done_callback(self.upload_result)

    def upload_result(self, future):
        result = future.result()
        if result.success:
            self.get_logger().info(
                f"Mission uploaded successfully ({result.wp_transfered} waypoints)"
            )
        else:
            self.get_logger().error("Mission upload FAILED")


def main():
    rclpy.init()
    node = EKFOriginMissionUploader()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
