import rclpy
from rclpy.node import Node
from manipulator_interfaces.msg import JointAngles, HomingStatus
from adafruit_pca9685 import PCA9685
import board
import busio

class ServoNode(Node):
    def __init__(self):
        super().__init__('servo_node')

        #PCA9685 setup
        i2c = busio.I2C(board.SCL, board.SDA)
        self.pca = PCA9685(i2c)
        self.pca.frequency = 50

        #servo channels
        self.channels = [0,3,7] #leg1,leg2,leg3

        #PWM range
        self.min_pulse = 500
        self.max_pulse = 2500

        #state 
        self.homed = False

        #subscribers
        self.homing_angles = self.create_subscription(
            JointAngles,
            'homing_angles',
            self.homing_angles_callback,
            10
        )
        self.joint_sub = self.create_subscription(
            JointAngles,
            'joint_angles',
            self.joint_callback,
            10
        )
        self.homing_sub = self.create_subscription(
            HomingStatus,
            'homing_status',
            self.homing_status_callback,
            10
        )

    def homing_angles_callback(self, msg):
        if not self.homed:
            self.set_servo_angle(self.channels[0], msg.alpha1)
            self.set_servo_angle(self.channels[1], msg.alpha2)
            self.set_servo_angle(self.channels[2], msg.alpha3)

    def homing_status_callback(self, msg):
        self.homed = msg.homed
        if self.homed:
            self.get_logger().info('Servos homed, ready for IK')

    def joint_callback(self, msg):
        if self.homed:
            self.set_servo_angle(0, msg.alpha1)
            self.set_servo_angle(3, msg.alpha2)
            self.set_servo_angle(7, msg.alpha3)
        else:
            self.get_logger().info('Not homed yet, ignoring IK command')

    def set_servo_angle(self, channel, angle):
        angle = max(-30, min(150.0, angle))
        pulse_us = self.max_pulse - (angle + 30)/180 * (self.max_pulse - self.min_pulse)
        duty_cycle = int(pulse_us/20000*65535)
        self.pca.channels[channel].duty_cycle = duty_cycle

def main(args=None):
    rclpy.init(args=args)
    servo_node = ServoNode()
    rclpy.spin(servo_node)
    servo_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()


        
