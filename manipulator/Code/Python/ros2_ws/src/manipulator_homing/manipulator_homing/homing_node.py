import rclpy
from rclpy.node import Node
from manipulator_interfaces.msg import HomingStatus
from gpiozero import Button
from adafruit_pca9685 import PCA9685
import board
import busio

class HomingNode(Node):
    def __init__(self):
        super().__init__('homing_node')

        # --- Parameters ---
        self.declare_parameter('homing_speed', 0.4)
        self.declare_parameter('switch_pin_1', 17)
        self.declare_parameter('switch_pin_2', 27)
        self.declare_parameter('switch_pin_3', 22)

        self.homing_speed = self.get_parameter('homing_speed').value
        pin1 = self.get_parameter('switch_pin_1').value
        pin2 = self.get_parameter('switch_pin_2').value
        pin3 = self.get_parameter('switch_pin_3').value

        # --- PCA9685 servo driver setup ---
        i2c = busio.I2C(board.SCL, board.SDA)
        self.pca = PCA9685(i2c)
        self.pca.frequency = 50  # standard 50Hz for servos

        # --- Servo channels on PCA9685 ---
        self.channels = [1, 4, 7]  # leg1, leg2, leg3

        # --- PWM pulse range in microseconds ---
        # These define the physical limits of your servos
        self.min_pulse = 500   # corresponds to -30 degrees
        self.max_pulse = 2500  # corresponds to +150 degrees

        # --- Limit switches (pull_up=True means normally HIGH, LOW when pressed) ---
        self.switch1 = Button(pin1, pull_up=True)
        self.switch2 = Button(pin2, pull_up=True)
        self.switch3 = Button(pin3, pull_up=True)

        # --- Angle state: where we THINK each servo is ---
        # We start assuming servos are near home (-30) to avoid
        # commanding a large jump on the first tick
        self.angle1 = 0.0
        self.angle2 = 0.0
        self.angle3 = 0.0

        # --- Homing state flags ---    
        self.homed1 = False
        self.homed2 = False
        self.homed3 = False

        # --- Offsets: difference between where switch fired and -30.0 ---
        # These get published so the servo node can apply them to IK commands. To be changed after homing
        self.angle1_offset = 0.0
        self.angle2_offset = 0.0
        self.angle3_offset = 0.0

        # --- Phase flags ---
        self.returning = False       # True once all switches hit, returning to 0
        self.homing_complete = False # True once back at 0, ready for IK

        # --- Publisher: tells servo node homing is done + sends offsets ---
        self.status_pub = self.create_publisher(HomingStatus, 'homing_status', 10)

        # --- Pre-check: if a switch is already pressed at startup ---
        # This happens if the node is restarted with legs already at home
        # We mark it homed immediately so the leg doesn't move at all
        if self.switch1.is_pressed: #not such a concern at present
            self.homed1 = True
            self.get_logger().warn('Switch 1 already pressed at startup — leg 1 skipped')
        if self.switch2.is_pressed:
            self.homed2 = True
            self.get_logger().warn('Switch 2 already pressed at startup — leg 2 skipped')
        if self.switch3.is_pressed:
            self.homed3 = True
            self.get_logger().warn('Switch 3 already pressed at startup — leg 3 skipped')

        # --- Send initial position to servos so they don't jump ---
        self._command_servos()

        # --- Start the homing loop at 50ms intervals ---
        self.timer = self.create_timer(0.05, self.homing_step)
        self.get_logger().info('Homing Node Started')

    def homing_step(self):
        # Do nothing once homing is complete
        if self.homing_complete:
            return

        if not self.returning:
            # === PHASE 1: Drive legs toward limit switches ===

            # Decrement angle for any leg not yet homed
            # This slowly moves each servo in the negative direction
            if not self.homed1:
                self.angle1 += self.homing_speed
                self.get_logger().info(f"angle1: {self.angle1}")
            if not self.homed2:
                self.angle2 += self.homing_speed
                self.get_logger().info(f"angle2: {self.angle2}")
            if not self.homed3:
                self.angle3 += self.homing_speed
                self.get_logger().info(f"angle3: {self.angle3}")

            # Command the servos to the new angles
            self._command_servos()

            # Check if any switch just got pressed
            # When a switch fires, record where it fired and stop decrementing
            # We do NOT snap the angle to -30 — we just stop and remember the offset
            if not self.homed1 and self.switch1.is_pressed:
                self.homed1 = True
                self.angle1_offset = self.angle1 - 20.0
                self.get_logger().info(f'Leg 1 homed at {self.angle1:.1f}° (offset={self.angle1_offset:.1f})')

            if not self.homed2 and self.switch2.is_pressed:
                self.homed2 = True
                self.angle2_offset = self.angle2 - 20.0
                self.get_logger().info(f'Leg 2 homed at {self.angle2:.1f}° (offset={self.angle2_offset:.1f})')

            if not self.homed3 and self.switch3.is_pressed:
                self.homed3 = True
                self.angle3_offset = self.angle3 - 20.0
                self.get_logger().info(f'Leg 3 homed at {self.angle3:.1f}° (offset={self.angle3_offset:.1f})')

            # Once all three are homed, start returning to 0
            if self.homed1 and self.homed2 and self.homed3:
                self.returning = True
                self.get_logger().info('All legs homed — returning to 0°')

        else:
            # === PHASE 2: Return all legs to 0 degrees ===
            # Increment each angle back toward 0
            if self.angle1 > (self.angle1_offset - 0.0):
                self.angle1 -= self.homing_speed
            if self.angle2 > (self.angle2_offset - 0.0):
                self.angle2 -= self.homing_speed
            if self.angle3 > (self.angle3_offset - 0.0):
                self.angle3 -= self.homing_speed

            # Command servos to updated angles
            self._command_servos()

            # Once all legs are back at 0, homing is complete
            if self.angle1 >= self.angle1_offset and self.angle2 >= self.angle2_offset and self.angle3 >= self.angle3_offset:
                # Snap exactly to 0
                self.angle1 = self.angle1_offset
                self.angle2 = self.angle2_offset
                self.angle3 = self.angle3_offset
                self._command_servos()

                self.homing_complete = True

                # Publish homing complete status with offsets for the servo node
                status = HomingStatus()
                status.homed = True
                status.offset1 = self.angle1_offset
                status.offset2 = self.angle2_offset
                status.offset3 = self.angle3_offset
                self.status_pub.publish(status)

                self.get_logger().info(
                    f'Homing complete. Offsets: '
                    f'leg1={self.angle1_offset:.1f} '
                    f'leg2={self.angle2_offset:.1f} '
                    f'leg3={self.angle3_offset:.1f}'
                )

    def _command_servos(self):
        """Send current angles directly to PCA9685."""
        self.set_servo_angle(self.channels[0], self.angle1)
        self.set_servo_angle(self.channels[1], self.angle2)
        self.set_servo_angle(self.channels[2], self.angle3)

    def set_servo_angle(self, channel, angle):
        """Convert angle in degrees to PWM and send to servo."""
        # Map angle to pulse width in microseconds
        pulse_us = self.min_pulse + (angle) / 180 * (self.max_pulse - self.min_pulse)
        # Convert microseconds to 16-bit duty cycle (period = 20000us)
        duty_cycle = int(pulse_us / 20000 * 65535)
        self.pca.channels[channel].duty_cycle = duty_cycle


def main(args=None):
    rclpy.init(args=args)
    homing_node = HomingNode()
    rclpy.spin(homing_node)
    homing_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()