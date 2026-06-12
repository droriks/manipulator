syms phi1 phi2 theta 

ct = cos(theta);
st = sin(theta);
cp1 = cos(phi1);
sp1 = sin(phi1);
cp2 = cos(phi2);
sp2 = sin(phi2);

%ball joint frame in bearing frame
T1 = [1 0 0 -4.33;
    0 cp2 -sp2 0;
    0 sp2 cp2 72.443;
    0 0 0 1];
%bearing frame in servo frame
T2 = [1 0 0 0;
    0 cp1 -sp1 0;
    0 sp1 cp1 51;
    0 0 0 1];

%servo frame in base frame
T3 = [1 0 0 12.53;
    0 ct -st 39.47;
    0 st ct 18.7;
    0 0 0 1];

T_actual = simplify(T3*T2*T1);

%whats the position without taking into account what i want it to be. just
%symbolically
p_actual = simplify(T_actual(1:3,4));

%these I control: orientation of platform and target z
nx_target = 0;
ny_target = 0;
z_target = 130;

%from that find the unit vector of platform normal
nz_target = sqrt(1 - nx_target^2 - ny_target^2);

%make it a vector
n = [nx_target; ny_target; nz_target];

%platform center is only affected by height
P_center = [0; 0; z_target];

%rotation matrix for the platform
%zP = n / norm(n);


%x_guess = [1; 0; 0];

%yP = cross(zP, x_guess);
%yP = yP / norm(yP);

%xP = cross(yP, zP);

rad = 0;
R_platform = [cos(rad) -sin(rad) 0;
    sin(rad) cos(rad) 0;
    0 0 1];
R_platform = R_platform/sqrt(3);

%rotation matrix for ball joint
rP = 70.6;
hBall = 26.81;

C1_local = [rP; 0; -hBall];

C2_local = [rP*cos(2*pi/3);
            rP*sin(2*pi/3);
            -hBall];

C3_local = [rP*cos(4*pi/3);
            rP*sin(4*pi/3);
            -hBall];

%ball position in the base frame
C1_base = P_center + R_platform*C1_local;
C2_base = P_center + R_platform*C2_local;
C3_base = P_center + R_platform*C3_local;


%get ball joint in target frame
Rz = @(g) [cos(g) -sin(g) 0;
           sin(g)  cos(g) 0;
           0       0      1];

gamma1 = 0;
gamma2 = 2*pi/3;
gamma3 = 4*pi/3;

C1_arm = Rz(-gamma1) * C1_base;
C2_arm = Rz(-gamma2) * C2_base;
C3_arm = Rz(-gamma3) * C3_base;

%solve equations
%theta 1. set the p_actual that has the thetas and whatnot to the
%calculated ball joint position
eqs1 = [
    p_actual(1) == C1_arm(1);
    p_actual(2) == C1_arm(2);
    p_actual(3) == C1_arm(3)
];

sol1 = vpasolve(eqs1, [theta, phi1, phi2], [.1, 0.1, 0.1]);

theta1 = double(sol1.theta);
phi1_1 = double(sol1.phi1);
phi2_1 = double(sol1.phi2);

theta1_deg = rad2deg(theta1)

%theta2 
eqs2 = [
    p_actual(1) == C2_arm(1);
    p_actual(2) == C2_arm(2);
    p_actual(3) == C2_arm(3)
];

sol2 = vpasolve(eqs2, [theta, phi1, phi2], [0, 0, 0]);

theta2 = double(sol2.theta);
phi1_2 = double(sol2.phi1);
phi2_2 = double(sol2.phi2);

theta2_deg = rad2deg(theta2)

%theta3
eqs3 = [
    p_actual(1) == C3_arm(1);
    p_actual(2) == C3_arm(2);
    p_actual(3) == C3_arm(3)
];

sol3 = vpasolve(eqs3, [theta, phi1, phi2], [0, 0, 0]);

theta3 = double(sol3.theta);
phi1_3 = double(sol3.phi1);
phi2_3 = double(sol3.phi2);

theta3_deg = rad2deg(theta3)






