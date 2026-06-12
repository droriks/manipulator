import rclpy
from rclpy.node import Node
from manipulator_interfaces.msg import Pose, JointAngles
from sympy import cos, sin, pi, nsolve, Matrix, symbols
import numpy as np


class IKNode(Node):
    def __init__(self):
        super().__init__('ik_node')
        self.subscription = self.create_subscription(
            Pose,
            'target_pose',
            self.pose_callback,
            10
        )
        self.publisher = self.create_publisher(JointAngles, 'joint_angles', 10)
        self.get_logger().info("IK Node started")


    def pose_callback(self, msg):
        self.get_logger().info(f"Received pose: H={msg.height}, psi={msg.psi}, theta={msg.theta}")
        alpha1, alpha2, alpha3 = self.solve_ik(msg.height, msg.psi, msg.theta)
        joint_angles = JointAngles()
        joint_angles.alpha1 = alpha1
        joint_angles.alpha2 = alpha2
        joint_angles.alpha3 = alpha3
        self.publisher.publish(joint_angles)
        self.get_logger().info(f"Publishing angles: {joint_angles.alpha1}, {joint_angles.alpha2}, {joint_angles.alpha3}")

    def solve_ik(self, h, psi, theta):
        # --- Constants ---
        Rp = 56.17
        Rb = 14.78
        Zb = 40.50
        L1 = 36
        L2 = 42.4
        del_h = 26.81   

        psi = psi*np.pi/180
        theta = theta*np.pi/180

        #syms the alphas and betas
        alpha1, beta1, alpha2, beta2, alpha3, beta3 = symbols('alpha1 beta1 alpha2 beta2 alpha3 beta3')

        # --- Basic Transformations and Arm 1 ---
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

        TCball = Matrix([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, del_h],
            [0, 0, 0, 1]
        ])

        Rxt = Matrix([
            [1, 0, 0],
            [0, cos(theta), -sin(theta)],
            [0, sin(theta), cos(theta)]
        ])

        Ryp = Matrix([
            [cos(psi), 0, sin(psi)],
            [0, 1, 0],
            [-sin(psi), 0, cos(psi)],
        ])

        R = Ryp*Rxt

        TCC = Matrix([
            [R[0,0], R[0,1], R[0,2], 0],
            [R[1,0], R[1,1], R[1,2], 0],
            [R[2,0], R[2,1], R[2,2], 0],
            [0, 0, 0, 1]
        ])

        TCD1 = Matrix([
            [1, 0, 0, -Rp],
            [0, 1, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])

        T1 = TOA1*TAB1*TBC1*TCball*TCC*TCD1

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

        T2 = TOO2*TOA2*TAB2*TBC2*TCC2*TCball*TCC*TCD2

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

        T3 = TOO3*TOA3*TAB3*TBC3*TCC3*TCball*TCC*TCD3

        # --- Solve for alphas and betas ---

        # Arm 1
        #extract eqns
        F11 = T1[0,3]
        F12 = T1[2,3] - h

        #numsolve
        solution1 = nsolve(
            [F11, F12],
            [alpha1, beta1],
            [0.3, 1.8]
        )

        alpha1deg = float(solution1[0]*180/np.pi)
        #beta1deg = float(solution1[1]*180/np.pi)

        # Arm 2
        #extract eqns
        F21 = T2[0,3]
        F22 = T2[2,3] - h

        #numsolve
        solution2 = nsolve(
            [F21, F22],
            [alpha2, beta2],
            [0.3, 1.8]
        )

        alpha2deg = float(solution2[0]*180/np.pi)
        #beta2deg = float(solution2[1]*180/np.pi)

        # Arm 3
        #extract eqns
        F31 = T3[0,3]
        F32 = T3[2,3] - h

        #numsolve
        solution3 = nsolve(
            [F31, F32],
            [alpha3, beta3],
            [0.3, 1.8]
        )

        alpha3deg = float(solution3[0]*180/np.pi)
        #beta3deg = float(solution3[1]*180/np.pi)

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