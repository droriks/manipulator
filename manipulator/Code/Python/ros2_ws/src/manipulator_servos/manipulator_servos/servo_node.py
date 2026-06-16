import rclpy
from rclpy.node import Node
from manipulator_interfaces.msg import JointAngles, HomingStatus
from adafruit_pca9685 import PCA9685
import board
import busio

class ServoNode(Node):
    def __init__(self):
        super().__init__('servo_node')

        # --- PCA9685 setup ---
        i2c = busio.I2C(board.SCL, board.SDA)
        self.pca = PCA9685(i2c)
        self.pca.frequency = 50

        # --- Servo channels on PCA9685 ---
        self.channels = [0, 4, 7]  # leg1, leg2, leg3

        # --- PWM pulse range in microseconds ---
        self.min_pulse = 500
        self.max_pulse = 2500

        # --- State ---
        self.homed = False
        # Offsets received from homing node — applied to every IK command
        # so the IK's reference frame matches the physical home position
        self.offsets = [0.0, 0.0, 0.0]

        # --- Subscribers ---

        # Homing status: fired once when homing completes
        # Contains homed=True and the three offsets
        self.homing_sub = self.create_subscription(
            HomingStatus,
            'homing_status',
            self.homing_status_callback,
            10
        )

        # IK joint angles: only acted on after homing is complete
        self.joint_sub = self.create_subscription(
            JointAngles,
            'joint_angles',
            self.joint_callback,
            10
        )

        self.get_logger().info('Servo Node Started — waiting for homing')

    def homing_status_callback(self, msg):
        """Called once when homing node finishes. Store offsets and go live."""
        if msg.homed:
            self.homed = True
            self.offsets = [msg.offset1, msg.offset2, msg.offset3]
            self.get_logger().info(
                f'Homing received. Offsets: '
                f'leg1={self.offsets[0]:.1f} '
                f'leg2={self.offsets[1]:.1f} '
                f'leg3={self.offsets[2]:.1f}'
            )
            self.get_logger().info('Ready for IK commands')

    def joint_callback(self, msg):
        """Called for every IK command. Apply offsets before commanding servos."""
        if not self.homed:
            self.get_logger().warn('Not homed yet — ignoring IK command')
            return

        # Subtract offset so IK angles map correctly to physical positions
        self.set_servo_angle(self.channels[0], msg.alpha1 - self.offsets[0])
        self.set_servo_angle(self.channels[1], msg.alpha2 - self.offsets[1])
        self.set_servo_angle(self.channels[2], msg.alpha3 - self.offsets[2])

    def set_servo_angle(self, channel, angle):
        """Convert angle in degrees to PWM and send to servo."""
        angle = max(-30.0, min(150.0, angle))
        pulse_us = self.max_pulse - (angle + 30) / 180 * (self.max_pulse - self.min_pulse)
        duty_cycle = int(pulse_us / 20000 * 65535)
        self.pca.channels[channel].duty_cycle = duty_cycle


def main(args=None):
    rclpy.init(args=args)
    servo_node = ServoNode()
    rclpy.spin(servo_node)
    servo_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()