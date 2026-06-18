import time
import numpy as np
from sympy import cos, sin, pi, Matrix, symbols, lambdify
from scipy.optimize import fsolve

# --- Constants ---
Rp = 56.17
Rb = 14.78
Zb = 40.50
L1 = 36
L2 = 42.4
del_h = 26.81

# --- Symbols ---
alpha1, beta1, alpha2, beta2, alpha3, beta3 = symbols('alpha1 beta1 alpha2 beta2 alpha3 beta3')
psi_sym, theta_sym = symbols('psi_sym theta_sym')

# --- Build symbolic F functions for all three arms, parameterized by psi/theta too ---

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

# Arm 1
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
F12_expr = T1[2,3]  # height term WITHOUT subtracting h yet — h is passed in at solve time

# Arm 2
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

# Arm 3
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

print("Compiling lambdify functions (one-time cost)...")
t_compile_start = time.time()

f11 = lambdify((alpha1, beta1, psi_sym, theta_sym), F11, 'numpy')
f12 = lambdify((alpha1, beta1, psi_sym, theta_sym), F12_expr, 'numpy')
f21 = lambdify((alpha2, beta2, psi_sym, theta_sym), F21, 'numpy')
f22 = lambdify((alpha2, beta2, psi_sym, theta_sym), F22_expr, 'numpy')
f31 = lambdify((alpha3, beta3, psi_sym, theta_sym), F31, 'numpy')
f32 = lambdify((alpha3, beta3, psi_sym, theta_sym), F32_expr, 'numpy')

print(f"Compilation took {time.time() - t_compile_start:.3f}s (one-time only)")


def solve_ik_fast(h, psi_deg, theta_deg):
    psi_rad = psi_deg * np.pi / 180
    theta_rad = theta_deg * np.pi / 180

    sol1 = fsolve(lambda x: [f11(x[0], x[1], psi_rad, theta_rad),
                              f12(x[0], x[1], psi_rad, theta_rad) - h],
                  [0.3, 1.8])
    sol2 = fsolve(lambda x: [f21(x[0], x[1], psi_rad, theta_rad),
                              f22(x[0], x[1], psi_rad, theta_rad) - h],
                  [0.3, 1.8])
    sol3 = fsolve(lambda x: [f31(x[0], x[1], psi_rad, theta_rad),
                              f32(x[0], x[1], psi_rad, theta_rad) - h],
                  [0.3, 1.8])

    alpha1_deg = sol1[0] * 180 / np.pi
    alpha2_deg = sol2[0] * 180 / np.pi
    alpha3_deg = sol3[0] * 180 / np.pi
    return alpha1_deg, alpha2_deg, alpha3_deg


# --- Test multiple poses, timing each solve individually ---
test_poses = [
    (110.0, 5.0, 0.0),    # your known-good baseline
    (90.0, 0.0, 0.0),     # near bottom of height range
    (125.0, 0.0, 0.0),    # near top of height range
    (107.5, 18.0, 0.0),   # near max psi tilt
    (107.5, 0.0, 18.0),   # near max theta tilt
    (107.5, 12.0, 12.0),  # combined tilt
]

print("\n--- Running test poses ---")
for h, psi_deg, theta_deg in test_poses:
    t0 = time.time()
    a1, a2, a3 = solve_ik_fast(h, psi_deg, theta_deg)
    elapsed = time.time() - t0
    print(f"h={h:.1f} psi={psi_deg:.1f} theta={theta_deg:.1f} -> "
          f"alphas=({a1:.3f}, {a2:.3f}, {a3:.3f})  [{elapsed:.5f}s]")