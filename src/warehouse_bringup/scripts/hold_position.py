import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class HoldPosition(Node):
    def __init__(self):
        super().__init__('hold_position')

        self.publisher = self.create_publisher(
            JointTrajectory,
            '/arm_controller/joint_trajectory',
            10
        )

        self.timer = self.create_timer(2.0, self.send_position)

    def send_position(self):
        msg = JointTrajectory()
        msg.joint_names = [
            'joint_1_s', 'joint_2_l', 'joint_3_u',
            'joint_4_r', 'joint_5_b', 'joint_6_t'
        ]

        point = JointTrajectoryPoint()
        point.positions = [0.0, -0.5, 0.5, 0.0, 0.8, 0.0]
        point.time_from_start.sec = 2

        msg.points.append(point)

        self.publisher.publish(msg)
        self.get_logger().info("Holding robot position")

def main():
    rclpy.init()
    node = HoldPosition()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
