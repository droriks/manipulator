import rclpy
from rclpy.node import Node
from manipulator_interfaces.msg import JointAngles
import csv
import os
from ament_index_python.packages import get_package_share_directory


class PlaybackNode(Node):
    def __init__(self):
        super().__init__('playback_node')


        default_path = os.path.join(
            get_package_share_directory('manipulator_ik'),
            'data',
            'sine_table.csv'
        )

        #parameter-- path to csv
        self.declare_parameter('csv_path', default_path)
        csv_path = self.get_parameter('csv_path').value

        self.angles = []
        with open(csv_path, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                self.angles.append((
                    float(row['alpha1']),
                    float(row['alpha2']),
                    float(row['alpha3'])
                ))

        self.get_logger().info(f"Loaded {len(self.angles)} rows from {csv_path}")

        self.joint_pub = self.create_publisher(JointAngles, 'joint_angles', 10)

        #playback state
        self.index = 0

        #timer
        self.timer = self.create_timer(0.01, self.timer_callback)
        self.get_logger().info("Playback started")

    def timer_callback(self):
        alpha1, alpha2, alpha3 = self.angles[self.index]

        msg = JointAngles()
        msg.alpha1 = alpha1
        msg.alpha2 = alpha2
        msg.alpha3 = alpha3
        self.joint_pub.publish(msg)

        self.index = (self.index + 1) % len(self.angles)


def main(args=None):
    rclpy.init(args=args)
    playback_node = PlaybackNode()
    rclpy.spin(playback_node)
    playback_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()