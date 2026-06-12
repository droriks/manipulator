clear; clc;

%% ============================================================
%  3-RRS inverse position kinematics
%  Goal:
%  Input: desired platform normal + platform height
%  Output: theta1, theta2, theta3 actuator angles
%
%  Coordinate convention:
%  Base frame:
%       origin at center of base
%       +Z upward
%       arm 1 lies along +X direction
%
%  Platform frame:
%       origin at platform center/reference plane
%       local +Z is platform normal
%
%  Leg points:
%       B_i = base revolute joint
%       C_i = middle revolute joint
%       A_i = spherical/ball joint on platform
%% ============================================================


%% ---------------- USER INPUTS ----------------

% Desired platform normal components
nx_target = 0.0;
ny_target = 0.0;

% Desired platform center height, mm
h = 158.88;

% Platform geometry, mm
% rP = distance from platform center to vertical line above ball joint
% hBall = vertical distance from platform plane down to ball center
rP = 70.6;
hBall = 26.81;

% Leg link lengths, mm
% CHANGE THESE if these are not the actual lower and upper link lengths.
Lbc = 51.0;   % lower link length: B_i to C_i
Lca = 72.58;     % upper link length: C_i to A_i

% Base geometry, mm
% rB = distance from base center to base revolute joint B_i.
% IMPORTANT: change this to your measured SolidWorks value.
rB = 45.44;   % CHANGE ME if base revolute joints are not at center

% Choose inverse kinematic branch:
% branch = 1 uses q_bi = alpha + beta - pi/2
% branch = 2 uses q_bi = beta - alpha - pi/2
branch = 1;


%% ---------------- PLATFORM NORMAL ----------------

% Compute nz from unit normal constraint
if nx_target^2 + ny_target^2 > 1
    error('Impossible normal: nx^2 + ny^2 must be <= 1');
end

nz_target = sqrt(1 - nx_target^2 - ny_target^2);

n = [nx_target; ny_target; nz_target];
n = n / norm(n);


%% ---------------- PLATFORM ROTATION MATRIX ----------------

% Platform z-axis is the desired normal
zP = n;

% Choose a reference direction for platform x-axis.
% This controls yaw/twist about the platform normal.
x_guess = [1; 0; 0];

% Safety check: if normal is too close to x_guess, use y instead
if abs(dot(zP, x_guess)) > 0.95
    x_guess = [0; 1; 0];
end

% Build an orthonormal right-handed platform frame
yP = cross(zP, x_guess);
yP = yP / norm(yP);

xP = cross(yP, zP);
xP = xP / norm(xP);

Rop = [xP yP zP];

% Platform center position in base frame
rp = [0; 0; h];


%% ---------------- PLATFORM BALL JOINT LOCATIONS ----------------

% Angles of the three platform joints around the platform
gamma1 = 0;
gamma2 = 2*pi/3;
gamma3 = 4*pi/3;
gammas = [gamma1, gamma2, gamma3];

% r_ai vectors: ball joint centers expressed in platform frame
ra1 = [rP*cos(gamma1); rP*sin(gamma1); -hBall];
ra2 = [rP*cos(gamma2); rP*sin(gamma2); -hBall];
ra3 = [rP*cos(gamma3); rP*sin(gamma3); -hBall];

ra = [ra1, ra2, ra3];


%% ---------------- BASE REVOLUTE JOINT LOCATIONS ----------------

% r_bi vectors: base revolute joint centers expressed in base frame
rb1 = [rB*cos(gamma1); rB*sin(gamma1); 0];
rb2 = [rB*cos(gamma2); rB*sin(gamma2); 0];
rb3 = [rB*cos(gamma3); rB*sin(gamma3); 0];

rb = [rb1, rb2, rb3];


%% ---------------- FIXED LOCAL BASE DIRECTIONS ----------------

% y'_bi directions.
% These are fixed directions in each leg's vertical plane.
% For a symmetric radial layout, use radial directions:
yprime1 = [cos(gamma1); sin(gamma1); 0];
yprime2 = [cos(gamma2); sin(gamma2); 0];
yprime3 = [cos(gamma3); sin(gamma3); 0];

yprime = [yprime1, yprime2, yprime3];


%% ---------------- SOLVE EACH LEG ----------------

theta = zeros(3,1);
alpha = zeros(3,1);
beta  = zeros(3,1);
gamma = zeros(3,1);

A_base = zeros(3,3);
Lba_vec = zeros(3,3);
Lba_len = zeros(3,1);

for i = 1:3

    % Ball joint A_i in base frame:
    % A_i = r_p + R_op*r_ai
    A_base(:,i) = rp + Rop*ra(:,i);

    % Vector from base revolute joint B_i to ball joint A_i
    % This is the paper's L_bai
    Lba_vec(:,i) = A_base(:,i) - rb(:,i);

    Lba_len(i) = norm(Lba_vec(:,i));

    % Reachability check
    if Lba_len(i) > Lbc + Lca
        error('Leg %d target is too far: |Lba| > Lbc + Lca', i);
    end

    if Lba_len(i) < abs(Lbc - Lca)
        error('Leg %d target is too close: |Lba| < |Lbc - Lca|', i);
    end

    % Law of cosines:
    % alpha_i = angle between L_bai and lower link L_bci
    cos_alpha = (Lbc^2 + Lba_len(i)^2 - Lca^2) / ...
                (2*Lbc*Lba_len(i));

    % gamma_i = elbow/internal angle between links
    cos_gamma = (Lca^2 + Lbc^2 - Lba_len(i)^2) / ...
                (2*Lca*Lbc);

    % Clamp to avoid tiny numerical errors outside [-1,1]
    cos_alpha = max(min(cos_alpha, 1), -1);
    cos_gamma = max(min(cos_gamma, 1), -1);

    alpha(i) = acos(cos_alpha);
    gamma(i) = acos(cos_gamma);

    % beta_i = angle between L_bai and y'_bi
    cos_beta = dot(Lba_vec(:,i), yprime(:,i)) / Lba_len(i);
    cos_beta = max(min(cos_beta, 1), -1);

    beta(i) = acos(cos_beta);

    % Two possible leg configurations, like elbow-up / elbow-down
    if branch == 1
        theta(i) = alpha(i) + beta(i) - pi/2;
    elseif branch == 2
        theta(i) = beta(i) - alpha(i) - pi/2;
    else
        error('branch must be 1 or 2');
    end
end


%% ---------------- RESULTS ----------------

theta_deg = rad2deg(theta);
alpha_deg = rad2deg(alpha);
beta_deg  = rad2deg(beta);
gamma_deg = rad2deg(gamma);

disp('Ball joint positions A_i in base frame [mm]:')
disp(A_base)

disp('Leg vectors L_bai = A_i - B_i [mm]:')
disp(Lba_vec)

disp('Leg lengths |L_bai| [mm]:')
disp(Lba_len)

disp('Actuator angles theta_i [deg]:')
disp(theta_deg)

disp('Intermediate angles [deg]:')
results_table = table(theta_deg, alpha_deg, beta_deg, gamma_deg, ...
    'VariableNames', {'theta_deg','alpha_deg','beta_deg','gamma_deg'});
disp(results_table)