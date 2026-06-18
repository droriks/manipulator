import rclpy
from rclpy.node import Node
from manipulator_interfaces.msg import Pose, HomingStatus
import math
import time


class SineWaveMotion(Node):
    def __init__(self):
        super().__init__('sine_wave_node')

        #parameters
        self.declare_parameter('height_center', 107.5)
        self.declare_parameter('height_amplitude', 0.0)
        self.declare_parameter('psi_center', 0.0)
        self.declare_parameter('psi_amplitude', 0.0)
        self.declare_parameter('theta_center', 0.0)
        self.declare_parameter('theta_amplitude', 0.0)
        self.declare_parameter('period', 2.0)
        self.declare_parameter('publish_rate', .009)

        #read parameter values into local variables
        self.height_center = self.get_parameter('height_center').value
        self.height_amplitude = self.get_parameter('height_amplitude').value
        self.psi_center = self.get_parameter('psi_center').value
        self.psi_amplitude = self.get_parameter('psi_amplitude').value
        self.theta_center = self.get_parameter('theta_center').value
        self.theta_amplitude = self.get_parameter('theta_amplitude').value
        self.period = self.get_parameter('period').value
        self.publish_rate = self.get_parameter('publish_rate').value

        #subscriber for starting
        self.homed = False
        self.homing_sub = self.create_subscription(
            HomingStatus,
            'homing_status',
            self.homing_callback,
            10
        )


        #publisher
        self.pose_pub = self.create_publisher(Pose, 'target_pose', 10)

        #time start record
        self.start_time = time.time()

        #start timer
        self.timer = self.create_timer(self.publish_rate, self.timer_callback)

        self.get_logger().info(
            f'Sine wave node start: period={self.period}s, '
            f'height = {self.height_center} +- {self.height_amplitude}, '
            f'psi = {self.psi_center} +- {self.psi_amplitude}, '
            f'theta = {self.theta_center} +- {self.theta_amplitude}'
        )

    def homing_callback(self, msg):
        if msg.homed and not self.homed:
            self.start_time = time.time()
        self.homed = msg.homed

    def timer_callback(self):
        if not self.homed:
            return
    
        elapsed_time = time.time() - self.start_time
        phase = 2 * math.pi * elapsed_time / self.period

        height = self.height_center + self.height_amplitude * math.sin(phase)
        psi = self.psi_center + self.psi_amplitude * math.sin(phase)
        theta = self.theta_center + self.theta_amplitude * math.sin(phase)

        msg = Pose()
        msg.height = height
        msg.psi = psi
        msg.theta = theta
        self.pose_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    sine_wave_node = SineWaveMotion()
    rclpy.spin(sine_wave_node)
    sine_wave_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()