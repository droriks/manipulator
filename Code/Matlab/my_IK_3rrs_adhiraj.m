function [alpha_deg_1, alpha_deg_2, alpha_deg_3, beta_deg_1, beta_deg_2, beta_deg_3] = my_IK_3rrs_adhiraj(Rb,L1,L2,Rp,H,psi,theta)


psi = psi*pi/180; %about z
theta = theta*pi/180; %about x

% first rotate about x, then rotate about z
Rx = [1 0 0;0 cos(theta) -sin(theta); 0 sin(theta) cos(theta)];
Rz = [cos(psi) -sin(psi) 0; sin(psi) cos(psi) 0; 0 0 1];
R = Rz*Rx; 

%desired position wrt O
T_DO_d = [R(1,1) R(1,2) R(1,3) 0; R(2,1) R(2,2) R(2,3) H; R(3,1) R(3,2) R(3,3) 0; 0 0 0 1];
P_DD = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1];
P_DO_d = T_DO_d*P_DD;

%Arm 1
%calculated position wrt O

syms alpha1;
syms beta1;

T_AO = [cos(alpha1) -sin(alpha1) 0 Rb; sin(alpha1) cos(alpha1) 0 0; 0 0 1 0; 0 0 0 1];
T_BA = [cos(pi-beta1) -sin(pi-beta1) 0 L1; sin(pi-beta1) cos(pi-beta1) 0 0; 0 0 1 0; 0 0 0 1];
T_CB = [cos(pi-(beta1-alpha1)) sin(pi-(beta1-alpha1)) 0 L2; -sin(pi-(beta1-alpha1)) cos(pi-(beta1-alpha1)) 0 0; 0 0 1 0; 0 0 0 1];
%C has same orientation as O axes, so to get Cp orientation, need R
T_CpC = [R(1,1) R(1,2) R(1,3) 0; R(2,1) R(2,2) R(2,3) 0; R(3,1) R(3,2) R(3,3) 0; 0 0 0 1];
T_DCp = [1 0 0 -Rp; 0 1 0 0; 0 0 1 0; 0 0 0 1];
P_DO_s = T_AO*T_BA*T_CB*T_CpC*T_DCp*P_DD;

%equate positions
eq1 = P_DO_s(1,4) - P_DO_d(1,4);
eq2 = P_DO_s(2,4) - P_DO_d(2,4);

E1 = matlabFunction(eq1,'Vars', [alpha1, beta1]);
E2 = matlabFunction(eq2,'Vars', [alpha1, beta1]);

fun = @(x) [E1(x(1), x(2));
            E2(x(1), x(2))];
x0 = [0.01; 0.01];
options = optimoptions('fsolve','Display','off');

[sol, fval, exitflag] = fsolve(fun, x0, options);

alpha_deg_1 = sol(1)*180/pi
beta_deg_1 = sol(2)*180/pi;

%Arm 2
%calculated position wrt O

syms alpha2;
syms beta2;

%rotate origina axis at O by 2pi/3 about y
T_OpO = [cos(2*pi/3) 0 sin(2*pi/3) 0; 0 1 0 0;-sin(2*pi/3) 0 cos(2*pi/3) 0; 0 0 0 1];
T_AOp = [cos(alpha2) -sin(alpha2) 0 Rb; sin(alpha2) cos(alpha2) 0 0; 0 0 1 0; 0 0 0 1];
T_BA = [cos(pi-beta2) -sin(pi-beta2) 0 L1; sin(pi-beta2) cos(pi-beta2) 0 0; 0 0 1 0; 0 0 0 1];
T_CB = [cos(pi-(beta2-alpha2)) sin(pi-(beta2-alpha2)) 0 L2; -sin(pi-(beta2-alpha2)) cos(pi-(beta2-alpha2)) 0 0; 0 0 1 0; 0 0 0 1];
%C has same orientation as Op axes, so to get back to O or Cp orientation
T_CpC = [cos(-2*pi/3) 0 sin(-2*pi/3) 0; 0 1 0 0;-sin(-2*pi/3) 0 cos(-2*pi/3) 0; 0 0 0 1];
%Cp has same orientation as O, now bring it to D's orientation or Cpp
T_CppCp = [R(1,1) R(1,2) R(1,3) 0; R(2,1) R(2,2) R(2,3) 0; R(3,1) R(3,2) R(3,3) 0; 0 0 0 1];
T_DCpp = [1 0 0 Rp*cos(pi/3); 0 1 0 0; 0 0 1 -Rp*sin(pi/3); 0 0 0 1];
P_DO_s = T_OpO*T_AOp*T_BA*T_CB*T_CpC*T_CppCp*T_DCpp*P_DD;

%equate positions
eq1 = P_DO_s(1,4) - P_DO_d(1,4);
eq2 = P_DO_s(2,4) - P_DO_d(2,4);

E1 = matlabFunction(eq1,'Vars', [alpha2, beta2]);
E2 = matlabFunction(eq2,'Vars', [alpha2, beta2]);

fun = @(x) [E1(x(1), x(2));
            E2(x(1), x(2))];
x0 = [0.01; 0.01];
options = optimoptions('fsolve','Display','off');

[sol, fval, exitflag] = fsolve(fun, x0, options);

alpha_deg_2 = sol(1)*180/pi
beta_deg_2 = sol(2)*180/pi;

%Arm 3

%calculated position wrt O

syms alpha3;
syms beta3;

%rotate origin axis at O by -2pi/3 about y
T_OpO = [cos(-2*pi/3) 0 sin(-2*pi/3) 0; 0 1 0 0;-sin(-2*pi/3) 0 cos(-2*pi/3) 0; 0 0 0 1];
T_AOp = [cos(alpha3) -sin(alpha3) 0 Rb; sin(alpha3) cos(alpha3) 0 0; 0 0 1 0; 0 0 0 1];
T_BA = [cos(pi-beta3) -sin(pi-beta3) 0 L1; sin(pi-beta3) cos(pi-beta3) 0 0; 0 0 1 0; 0 0 0 1];
T_CB = [cos(pi-(beta3-alpha3)) sin(pi-(beta3-alpha3)) 0 L2; -sin(pi-(beta3-alpha3)) cos(pi-(beta3-alpha3)) 0 0; 0 0 1 0; 0 0 0 1];
%C has same orientation as Op axes, so to get back to O or Cp orientation
T_CpC = [cos(2*pi/3) 0 sin(2*pi/3) 0; 0 1 0 0;-sin(2*pi/3) 0 cos(2*pi/3) 0; 0 0 0 1];
%Cp has same orientation as O, now bring it to D's orientation or Cpp
T_CppCp = [R(1,1) R(1,2) R(1,3) 0; R(2,1) R(2,2) R(2,3) 0; R(3,1) R(3,2) R(3,3) 0; 0 0 0 1];
T_DCpp = [1 0 0 Rp*cos(pi/3); 0 1 0 0; 0 0 1 Rp*sin(pi/3); 0 0 0 1];
P_DO_s = T_OpO*T_AOp*T_BA*T_CB*T_CpC*T_CppCp*T_DCpp*P_DD;

%equate positions
eq1 = P_DO_s(1,4) - P_DO_d(1,4);
eq2 = P_DO_s(2,4) - P_DO_d(2,4);

E1 = matlabFunction(eq1,'Vars', [alpha3, beta3]);
E2 = matlabFunction(eq2,'Vars', [alpha3, beta3]);

fun = @(x) [E1(x(1), x(2));
            E2(x(1), x(2))];
x0 = [0.01; 0.01];
options = optimoptions('fsolve','Display','off');

[sol, fval, exitflag] = fsolve(fun, x0, options);

alpha_deg_3 = sol(1)*180/pi
beta_deg_3 = sol(2)*180/pi;