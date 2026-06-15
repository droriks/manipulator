import rclpy
from rclpy.node import Node
from manipulator_interfaces.msg import JointAngles, HomingStatus
from gpiozero import Button

class HomingNode(Node):
    def __init__(self):
        super().__init__('homing_node')

        #parameters and pins
        self.declare_parameter('homing_speed', .5)
        self.declare_parameter('switch_pin_1', 17)
        self.declare_parameter('switch_pin_2', 27)
        self.declare_parameter('switch_pin_3', 22)

        self.homing_speed = self.get_parameter('homing_speed').value
        pin1 = self.get_parameter('switch_pin_1').value
        pin2 = self.get_parameter('switch_pin_2').value
        pin3 = self.get_parameter('switch_pin_3').value

        #publishers
        self.joint_pub = self.create_publisher(JointAngles, 'homing_angles', 10)
        self.status_pub = self.create_publisher(HomingStatus, 'homing_status', 10)

        #limit switches
        self.switch1 = Button(pin1, pull_up=True)
        self.switch2 = Button(pin2, pull_up=True)
        self.switch3 = Button(pin3, pull_up=True)

        #states
        self.angle1 = 0.0
        self.angle2 = 0.0
        self.angle3 = 0.0
        self.homed1 = False
        self.homed2 = False
        self.homed3 = False
        self.returning = False
        self.homing_complete = False

        #timer
        self.timer = self.create_timer(.05, self.homing_step)
        self.get_logger().info('Homing Node Started')

    def homing_step(self):
        if self.homing_complete == True:
            return

        if self.returning == False:
            #drive each servo negative until switch hit
            if not self.homed1:
                self.angle1 -= self.homing_speed
            if not self.homed2:
                self.angle2 -= self.homing_speed
            if not self.homed3:
                self.angle3 -= self.homing_speed
        
            #check limit switches
            if not self.homed1 and self.switch1.is_pressed:
                self.homed1 = True
                self.angle1 = -30.0
                self.get_logger().info('Leg 1 Homed')
            if not self.homed2 and self.switch2.is_pressed:
                self.homed2 = True
                self.angle2 = -30.0
                self.get_logger().info('Leg 2 Homed')
            if not self.homed3 and self.switch3.is_pressed:
                self.homed3 = True
                self.angle3 = -30.0
                self.get_logger().info('Leg 3 Homed')

            if self.homed1 and self.homed2 and self.homed3:
                self.returning = True
                self.get_logger().info('All legs homed, returning to 0') 
        
        else: #meaning, its returning
            if self.angle1 < 0.0:
                self.angle1 += self.homing_speed
            if self.angle2 < 0.0:
                self.angle2 += self.homing_speed
            if self.angle3 < 0.0:
                self.angle3 += self.homing_speed

            #they got there
            if self.angle1 >= 0 and self.angle2 >= 0 and self.angle3 >= 0:
                self.angle1 = 0.0
                self.angle2 = 0.0
                self.angle3 = 0.0
                self.homing_complete = True
                status = HomingStatus()
                status.homed = True
                self.status_pub.publish(status)
                self.get_logger().info('Homing Complete')
        
        msg = JointAngles()
        msg.alpha1 = self.angle1
        msg.alpha2 = self.angle2
        msg.alpha3 = self.angle3
        self.joint_pub.publish(msg)

def main(args=None):
    #create node
    rclpy.init(args=args)
    homing_node = HomingNode()
    
    #use node
    rclpy.spin(homing_node)

    #destroy node
    homing_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
        