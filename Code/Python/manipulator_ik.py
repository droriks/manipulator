import numpy as np
from sympy import symbols, cos, sin, pi, solve, nsolve, Matrix


def main():
    alpha1deg, alpha2deg, alpha3deg = find_alphas()
    print(alpha1deg, alpha2deg, alpha3deg)

def find_alphas():
    # --- Set Parameters ---
    h = 110 #height
    psi = 10 #z
    theta = 0 #x

    #change to radians
    psi = psi*np.pi/180
    theta = theta*np.pi/180

    # --- Constants ---
    Rp = 70.6
    Rb = 42.114
    L1 = 51
    L2 = 72.43
    del_h = 26.66

    #syms the alphas and betas
    alpha1, beta1, alpha2, beta2, alpha3, beta3 = symbols('alpha1 beta1 alpha2 beta2 alpha3 beta3')

    # --- Basic Transformations and Arm 1 ---
    TOA1 = Matrix([
        [cos(alpha1), -sin(alpha1), 0, Rb],
        [sin(alpha1), cos(alpha1), 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])

    TAB1 = Matrix([
        [cos(pi-beta1), -sin(pi-beta1), 0, L1],
        [sin(pi-beta1), cos(pi-beta1), 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])

    TBC1 = Matrix([
        [cos(-(pi-beta1+alpha1)), -sin(-(pi-beta1+alpha1)), 0, L2],
        [sin(-(pi-beta1+alpha1)), cos(-(pi-beta1+alpha1)), 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])

    TCball = Matrix([
        [1, 0, 0, 0],
        [0, 1, 0, del_h],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])

    Rxt = Matrix([
        [1, 0, 0],
        [0, cos(theta), -sin(theta)],
        [0, sin(theta), cos(theta)]
    ])

    Rzp = Matrix([
        [cos(psi), -sin(psi), 0],
        [sin(psi), cos(psi), 0],
        [0, 0, 1]
    ])

    R = Rzp*Rxt

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
        [cos(2*pi/3), 0, sin(2*pi/3), 0],
        [0, 1, 0, 0],
        [-sin(2*pi/3), 0, cos(2*pi/3), 0],
        [0, 0, 0, 1]
    ])

    TOA2 = Matrix([
        [cos(alpha2), -sin(alpha2), 0, Rb],
        [sin(alpha2), cos(alpha2), 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])

    TAB2 = Matrix([
        [cos(pi-beta2), -sin(pi-beta2), 0, L1],
        [sin(pi-beta2), cos(pi-beta2), 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])

    TBC2 = Matrix([
        [cos(-(pi-beta2+alpha2)), -sin(-(pi-beta2+alpha2)), 0, L2],
        [sin(-(pi-beta2+alpha2)), cos(-(pi-beta2+alpha2)), 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])

    TCC2 = Matrix([
        [cos(-2*pi/3), 0, sin(-2*pi/3), 0],
        [0, 1, 0, 0],
        [-sin(-2*pi/3), 0, cos(-2*pi/3), 0],
        [0, 0, 0, 1]
    ])


    TCD2 = Matrix([
        [1, 0, 0, Rp*cos(pi/3)],
        [0, 1, 0, 0],
        [0, 0, 1, Rp*sin(pi/3)],
        [0, 0, 0, 1]
    ])

    T2 = TOO2*TOA2*TAB2*TBC2*TCC2*TCball*TCC*TCD2

    # --- Arm 3 ---
    TOO3 = Matrix([
        [cos(-2*pi/3), 0, sin(-2*pi/3), 0],
        [0, 1, 0, 0],
        [-sin(-2*pi/3), 0, cos(-2*pi/3), 0],
        [0, 0, 0, 1]
    ])

    TOA3 = Matrix([
        [cos(alpha3), -sin(alpha3), 0, Rb],
        [sin(alpha3), cos(alpha3), 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])

    TAB3 = Matrix([
        [cos(pi-beta3), -sin(pi-beta3), 0, L1],
        [sin(pi-beta3), cos(pi-beta3), 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])

    TBC3 = Matrix([
        [cos(-(pi-beta3+alpha3)), -sin(-(pi-beta3+alpha3)), 0, L2],
        [ sin(-(pi-beta3+alpha3)), cos(-(pi-beta3+alpha3)), 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])

    TCC3 = Matrix([
        [cos(2*pi/3), 0, sin(2*pi/3), 0],
        [0, 1, 0, 0],
        [-sin(2*pi/3), 0, cos(2*pi/3), 0],
        [0, 0, 0, 1]
    ])

    TCD3 = Matrix([
        [1, 0, 0, Rp*cos(pi/3)],
        [0, 1, 0, 0],
        [0, 0, 1, -Rp*sin(pi/3)],
        [0, 0, 0, 1]
    ])

    T3 = TOO3*TOA3*TAB3*TBC3*TCC3*TCball*TCC*TCD3

    # --- Solve for alphas and betas ---

    # Arm 1
    #extract eqns
    F11 = T1[0,3]
    F12 = T1[1,3] - h

    #numsolve
    solution1 = nsolve(
        [F11, F12],
        [alpha1, beta1],
        [0.5, 1]
    )

    alpha1deg = float(solution1[0]*180/np.pi)
    beta1deg = float(solution1[1]*180/np.pi)

    # Arm 2
    #extract eqns
    F21 = T2[0,3]
    F22 = T2[1,3] - h

    #numsolve
    solution2 = nsolve(
        [F21, F22],
        [alpha2, beta2],
        [0.5, 1]
    )

    alpha2deg = float(solution2[0]*180/np.pi)
    beta2deg = float(solution2[1]*180/np.pi)

    # Arm 3
    #extract eqns
    F31 = T3[0,3]
    F32 = T3[1,3] - h

    #numsolve
    solution3 = nsolve(
        [F31, F32],
        [alpha3, beta3],
        [0.5, 1]
    )

    alpha3deg = float(solution3[0]*180/np.pi)
    beta3deg = float(solution3[1]*180/np.pi)

    return alpha1deg, alpha2deg, alpha3deg
    

if __name__ == "__main__":
    main()