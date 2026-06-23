import rclpy
from rclpy.node import Node
from manipulator_interfaces.msg import Pose, HomingStatus
import pygame

WINDOW_SIZE = 400
GRID_DIVISIONS = 10
MAX_ANGLE_DEG = 20.0
HEIGHT = 107.5
HEIGHT_MIN = 90.0
HEIGHT_MAX = 125.0
HEIGHT_RATE = 10.0  # mm/s while holding arrow key


class JoystickNode(Node):
    def __init__(self):
        super().__init__('joystick_node')

        self.declare_parameter('publish_rate', 0.01)
        self.publish_rate = self.get_parameter('publish_rate').value

        self.homed = False
        self.homing_sub = self.create_subscription(
            HomingStatus,
            'homing_status',
            self.homing_callback,
            10
        )

        self.pose_pub = self.create_publisher(Pose, 'target_pose', 10)

        pygame.init()
        self.screen = pygame.display.set_mode((WINDOW_SIZE, WINDOW_SIZE))
        pygame.display.set_caption('Joystick Control')
        self.font = pygame.font.SysFont(None, 24)
        self.last_psi = 0.0
        self.last_theta = 0.0
        self.height = HEIGHT

        self.timer = self.create_timer(self.publish_rate, self.timer_callback)

        self.get_logger().info('Joystick node started')

    def homing_callback(self, msg):
        self.homed = msg.homed

    def draw_grid(self):
        self.screen.fill((255, 255, 255))
        step = WINDOW_SIZE // GRID_DIVISIONS
        for i in range(0, WINDOW_SIZE + 1, step):
            pygame.draw.line(self.screen, (200, 200, 200), (i, 0), (i, WINDOW_SIZE))
            pygame.draw.line(self.screen, (200, 200, 200), (0, i), (WINDOW_SIZE, i))
        pygame.draw.rect(self.screen, (0, 0, 0), (0, 0, WINDOW_SIZE, WINDOW_SIZE), 2)

    def draw_readout(self):
        text = self.font.render(
            f'psi: {self.last_psi:.1f}  theta: {self.last_theta:.1f}  height: {self.height:.1f}',
            True, (0, 0, 0)
        )
        rect = text.get_rect()
        rect.bottomleft = (5, WINDOW_SIZE - 5)
        pygame.draw.rect(self.screen, (255, 255, 255), rect.inflate(6, 6))
        self.screen.blit(text, rect)

    def timer_callback(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.get_logger().info('Joystick window closed, shutting down')
                self.destroy_node()
                rclpy.shutdown()
                return

        self.draw_grid()

        keys = pygame.key.get_pressed()
        if keys[pygame.K_UP]:
            self.height = min(self.height + HEIGHT_RATE * self.publish_rate, HEIGHT_MAX)
        if keys[pygame.K_DOWN]:
            self.height = max(self.height - HEIGHT_RATE * self.publish_rate, HEIGHT_MIN)

        if self.homed:
            x, y = pygame.mouse.get_pos()
            if 0 <= x <= WINDOW_SIZE and 0 <= y <= WINDOW_SIZE:
                self.last_psi = (x / WINDOW_SIZE * 2.0 - 1.0) * MAX_ANGLE_DEG
                self.last_theta = (1.0 - y / WINDOW_SIZE * 2.0) * MAX_ANGLE_DEG

                msg = Pose()
                msg.height = self.height
                msg.psi = self.last_psi
                msg.theta = self.last_theta
                self.pose_pub.publish(msg)

        self.draw_readout()
        pygame.display.flip()


def main(args=None):
    rclpy.init(args=args)
    joystick_node = JoystickNode()
    try:
        rclpy.spin(joystick_node)
    except KeyboardInterrupt:
        pass
    finally:
        pygame.quit()
        if rclpy.ok():
            joystick_node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
