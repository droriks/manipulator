import rclpy
from rclpy.node import Node
from manipulator_interfaces.msg import Pose, JointAngles, HomingStatus
from sympy import cos, sin, pi, lambdify, Matrix, symbols
from scipy.optimize import fsolve
import numpy as np


class IKNode(Node):
    def __init__(self):
        super().__init__('ik_node')
        self.homed = False
        self.target_sub = self.create_subscription(
            Pose,
            'target_pose',
            self.pose_callback,
            10
        )
        self.homing_sub = self.create_subscription(
            HomingStatus,
            'homing_status',
            self.homing_callback,
            10
        )

        self.publisher = self.create_publisher(JointAngles, 'joint_angles', 10)

        self._build_ik_functions()

        self.get_logger().info("IK Node started")

    def _build_ik_functions(self):
        # --- Constants ---
        Rp = 56.17
        Rb = 14.78
        Zb = 40.50
        L1 = 36
        L2 = 42.4
        del_h = 26.81

        alpha1, beta1, alpha2, beta2, alpha3, beta3 = symbols('alpha1 beta1 alpha2 beta2 alpha3 beta3')
        psi_sym, theta_sym = symbols('psi_sym theta_sym')

        Rxt = Matrix([
            [1, 0, 0],
            [0, cos(theta_sym), -sin(theta_sym)],
            [0, sin(theta_sym), cos(theta_sym)]
        ])

        Ryp = Matrix([
            [cos(psi_sym), 0, sin(psi_sym)],
            [0, 1, 0],
            [-sin(psi_sym), 0, cos(psi_sym)],
        ])

        R = Ryp * Rxt

        TCC = Matrix([
            [R[0,0], R[0,1], R[0,2], 0],
            [R[1,0], R[1,1], R[1,2], 0],
            [R[2,0], R[2,1], R[2,2], 0],
            [0, 0, 0, 1]
        ])

        TCball = Matrix([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, del_h],
            [0, 0, 0, 1]
        ])

         # --- Arm 1 ---
        TOA1 = Matrix([
            [cos(-(alpha1)), 0, sin(-(alpha1)), Rb],
            [0, 1, 0, 0],
            [-sin(-(alpha1)), 0, cos(-(alpha1)), Zb],
            [0, 0, 0, 1]
        ])

        TAB1 = Matrix([
            [cos(-(pi-beta1)), 0, sin(-(pi-beta1)), L1],
            [0, 1, 0, 0],
            [-sin(-(pi-beta1)), 0, cos(-(pi-beta1)), 0],
            [0, 0, 0, 1]
        ])

        TBC1 = Matrix([
            [cos(pi-beta1+alpha1), 0, sin(pi-beta1+alpha1), L2],
            [0, 1, 0, 0],
            [-sin(pi-beta1+alpha1), 0, cos(pi-beta1+alpha1), 0],
            [0, 0, 0, 1]
        ])

        TCD1 = Matrix([
            [1, 0, 0, -Rp],
            [0, 1, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])

        T1 = TOA1 * TAB1 * TBC1 * TCball * TCC * TCD1

        F11 = T1[0,3]
        F12_expr = T1[2,3]
        
        # --- Arm 2 ---
        TOO2 = Matrix([
            [cos(2*pi/3), -sin(2*pi/3), 0, 0],
            [sin(2*pi/3), cos(2*pi/3), 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])

        TOA2 = Matrix([
            [cos(-(alpha2)), 0, sin(-(alpha2)), Rb],
            [0, 1, 0, 0],
            [-sin(-(alpha2)), 0, cos(-(alpha2)), Zb],
            [0, 0, 0, 1]
        ])

        TAB2 = Matrix([
            [cos(-(pi-beta2)), 0, sin(-(pi-beta2)), L1],
            [0, 1, 0, 0],
            [-sin(-(pi-beta2)), 0, cos(-(pi-beta2)), 0],
            [0, 0, 0, 1]
        ])

        TBC2 = Matrix([
            [cos(pi-beta2+alpha2), 0, sin(pi-beta2+alpha2), L2],
            [0, 1, 0, 0],
            [-sin(pi-beta2+alpha2), 0, cos(pi-beta2+alpha2), 0],
            [0, 0, 0, 1]
        ])

        TCC2 = Matrix([
            [cos(-2*pi/3), -sin(-2*pi/3), 0, 0],
            [sin(-2*pi/3), cos(-2*pi/3), 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])

        TCD2 = Matrix([
            [1, 0, 0, Rp*cos(pi/3)],
            [0, 1, 0, -Rp*sin(pi/3)],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])

        T2 = TOO2 * TOA2 * TAB2 * TBC2 * TCC2 * TCball * TCC * TCD2

        F21 = T2[0,3]
        F22_expr = T2[2,3]

        # --- Arm 3 ---
        TOO3 = Matrix([
            [cos(-2*pi/3), -sin(-2*pi/3), 0, 0],
            [sin(-2*pi/3), cos(-2*pi/3), 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])

        TOA3 = Matrix([
            [cos(-(alpha3)), 0, sin(-(alpha3)), Rb],
            [0, 1, 0, 0],
            [-sin(-(alpha3)), 0, cos(-(alpha3)), Zb],
            [0, 0, 0, 1]
        ])

        TAB3 = Matrix([
            [cos(-(pi-beta3)), 0, sin(-(pi-beta3)), L1],
            [0, 1, 0, 0],
            [-sin(-(pi-beta3)), 0, cos(-(pi-beta3)), 0],
            [0, 0, 0, 1]
        ])

        TBC3 = Matrix([
            [cos(pi-beta3+alpha3), 0, sin(pi-beta3+alpha3), L2],
            [0, 1, 0, 0],
            [-sin(pi-beta3+alpha3), 0, cos(pi-beta3+alpha3), 0],
            [0, 0, 0, 1]
        ])

        TCC3 = Matrix([
            [cos(2*pi/3), -sin(2*pi/3), 0, 0],
            [sin(2*pi/3), cos(2*pi/3), 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])

        TCD3 = Matrix([
            [1, 0, 0, Rp*cos(pi/3)],
            [0, 1, 0, Rp*sin(pi/3)],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])

        T3 = TOO3 * TOA3 * TAB3 * TBC3 * TCC3 * TCball * TCC * TCD3

        F31 = T3[0,3]
        F32_expr = T3[2,3]

        self.f11 = lambdify((alpha1, beta1, psi_sym, theta_sym), F11, 'numpy')
        self.f12 = lambdify((alpha1, beta1, psi_sym, theta_sym), F12_expr, 'numpy')
        self.f21 = lambdify((alpha2, beta2, psi_sym, theta_sym), F21, 'numpy')
        self.f22 = lambdify((alpha2, beta2, psi_sym, theta_sym), F22_expr, 'numpy')
        self.f31 = lambdify((alpha3, beta3, psi_sym, theta_sym), F31, 'numpy')
        self.f32 = lambdify((alpha3, beta3, psi_sym, theta_sym), F32_expr, 'numpy')


    def homing_callback(self, msg):
        self.homed = msg.homed

    def pose_callback(self, msg):
        if not self.homed:
            self.get_logger().warn("Pose not sent -- not homed yet")
            return
        self.get_logger().info(f"Received pose: H={msg.height}, psi={msg.psi}, theta={msg.theta}")
        
        import time
        t1 = time.time()
        alpha1, alpha2, alpha3 = self.solve_ik_fast(msg.height, msg.psi, msg.theta)
        self.get_logger().info(f"solve_ik took {time.time() - t1:.3f}s")
        
        #alpha1, alpha2, alpha3 = self.solve_ik(msg.height, msg.psi, msg.theta)
        joint_angles = JointAngles()
        joint_angles.alpha1 = alpha1
        joint_angles.alpha2 = alpha2
        joint_angles.alpha3 = alpha3
        self.publisher.publish(joint_angles)
        self.get_logger().info(f"Publishing angles: {joint_angles.alpha1}, {joint_angles.alpha2}, {joint_angles.alpha3}")

    def solve_ik_fast(self, h, psi_deg, theta_deg):
            import time
            t0 = time.time()

            psi_rad = psi_deg*np.pi/180
            theta_rad = theta_deg*np.pi/180
            
            sol1 = fsolve(lambda x: [self.f11(x[0], x[1], psi_rad, theta_rad),
                                        self.f12(x[0], x[1], psi_rad, theta_rad) - h],
                                        [0.3, 1.8])
            sol2 = fsolve(lambda x: [self.f21(x[0], x[1], psi_rad, theta_rad),
                                        self.f22(x[0], x[1], psi_rad, theta_rad) - h],
                                        [0.3, 1.8])
            sol3 = fsolve(lambda x: [self.f31(x[0], x[1], psi_rad, theta_rad),
                                        self.f32(x[0], x[1], psi_rad, theta_rad) - h],
                                        [0.3, 1.8])

            self.get_logger().info(f"actual fsolve work took {time.time() - t0:.4f}s")
            
            alpha1deg = sol1[0] * 180 / np.pi
            alpha2deg = sol2[0] * 180 / np.pi
            alpha3deg = sol3[0] * 180 / np.pi

            return alpha1deg, alpha2deg, alpha3deg
    

def main(args=None):
    #create
    rclpy.init(args=args)
    ik_node = IKNode()
    #use
    rclpy.spin(ik_node)
    #destroy
    ik_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()