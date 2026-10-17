#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from px4_msgs.msg import VehicleImu
from sensor_msgs.msg import Imu
from std_msgs.msg import Header

class ImuBridge(Node):
    def __init__(self):
        super().__init__('imu_bridge')

        # BEST_EFFORT QoS 설정 (PX4 RTPS 퍼블리셔와 호환)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # VehicleImu 구독
        self.sub = self.create_subscription(
            VehicleImu,
            '/fmu/out/vehicle_imu',
            self.imu_callback,
            qos_profile
        )

        # sensor_msgs/Imu 퍼블리셔
        self.pub = self.create_publisher(Imu, '/imu/data/raw', 10)

        self.get_logger().info('IMU Bridge started (VehicleImu → sensor_msgs/Imu)')

    def imu_callback(self, msg: VehicleImu):
        imu_msg = Imu()
        imu_msg.header = Header()
        # 시뮬레이터 환경에 맞게 현재 시간을 사용합니다.
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = "base_link"
        # ned 
        # delta_angle → angular velocity [rad/s]
        #dt_angle = msg.delta_angle_dt * 1e-6  # µs → s
        #if dt_angle > 0:
        #    imu_msg.angular_velocity.x = float(msg.delta_angle[0]) / dt_angle
        #    imu_msg.angular_velocity.y = float(msg.delta_angle[1]) / dt_angle
        #    imu_msg.angular_velocity.z = float(msg.delta_angle[2]) / dt_angle

        # delta_velocity → linear acceleration [m/s²]
        #dt_vel = msg.delta_velocity_dt * 1e-6
        #if dt_vel > 0:
        #    imu_msg.linear_acceleration.x = float(msg.delta_velocity[0]) / dt_vel
        #    imu_msg.linear_acceleration.y = float(msg.delta_velocity[1]) / dt_vel
        #    imu_msg.linear_acceleration.z = float(msg.delta_velocity[2]) / dt_vel

        #enu    
        # delta_angle → angular velocity [rad/s]
        dt_angle = msg.delta_angle_dt * 1e-6  # µs → s
        if dt_angle > 0:
            imu_msg.angular_velocity.x = float(msg.delta_angle[1]) / dt_angle  # y → x
            imu_msg.angular_velocity.y = float(msg.delta_angle[0]) / dt_angle  # x → y
            imu_msg.angular_velocity.z = -float(msg.delta_angle[2]) / dt_angle # -z

        # delta_velocity → linear acceleration [m/s²]
        dt_vel = msg.delta_velocity_dt * 1e-6
        if dt_vel > 0:
            imu_msg.linear_acceleration.x = float(msg.delta_velocity[1]) / dt_vel  # y → x
            imu_msg.linear_acceleration.y = float(msg.delta_velocity[0]) / dt_vel  # x → y
            imu_msg.linear_acceleration.z = -float(msg.delta_velocity[2]) / dt_vel # -z

            

        # 공분산 값 추가 (orientation은 측정되지 않으므로 첫 원소에 -1)
        imu_msg.orientation_covariance = [-1.0] + [0.0] * 8
        imu_msg.angular_velocity_covariance = [
            0.02, 0.0, 0.0,
            0.0, 0.02, 0.0,
            0.0, 0.0, 0.02
        ]
        imu_msg.linear_acceleration_covariance = [
            0.04, 0.0, 0.0,
            0.0, 0.04, 0.0,
            0.0, 0.0, 0.04
        ]

        self.pub.publish(imu_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ImuBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
