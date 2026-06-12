function [alpha_deg, beta_deg, A, B, C, O, P] = IK_3RRS(Rb,Rp,L1,L2,h, psi, theta)
%% ---------------- PARAMETERS ----------------
psi   = deg2rad(psi);
theta = deg2rad(theta);
%% ---------------- ROTATION ----------------
Rz = [cos(psi) -sin(psi) 0;
      sin(psi)  cos(psi) 0;
      0 0 1];
Rx = [1 0 0;
      0 cos(theta) -sin(theta);
      0 sin(theta) cos(theta)];
R = Rz * Rx;
P_center = [0; h; 0];
%% ---------------- BASE ----------------
O = zeros(3,3);
O(:,1) = [Rb; 0; 0];
O(:,2) = [Rb*cos(-2*pi/3); 0; Rb*sin(-2*pi/3)];
O(:,3) = [Rb*cos( 2*pi/3); 0; Rb*sin( 2*pi/3)];
%% ---------------- PLATFORM ----------------
P_local = zeros(3,3);
P_local(:,1) = [Rp; 0; 0];
P_local(:,2) = [Rp*cos(-2*pi/3); 0; Rp*sin(-2*pi/3)];
P_local(:,3) = [Rp*cos( 2*pi/3); 0; Rp*sin( 2*pi/3)];
P = R * P_local + P_center;
%% ---------------- STORAGE ----------------
A = zeros(3,3);
B = zeros(3,3);
C = zeros(3,3);
alpha = zeros(1,3);
beta  = zeros(1,3);
%% ---------------- IK ----------------
for i = 1:3
    Oi = O(:,i);
    Ci = P(:,i);
    OC = Ci - Oi;
    d = norm(OC);
if d > (L1 + L2) || d < abs(L1 - L2)
        error('Unreachable pose at leg %d', i);
end
    e = OC / d;
    %% elbow solution
    x = (L1^2 - L2^2 + d^2) / (2*d);
    h_elbow = sqrt(max(L1^2 - x^2, 0));
    OA_dir = Oi / norm(Oi);
    n = cross(OA_dir, e);
if norm(n) < 1e-12
        n = cross([0;1;0], e);
end
    n = n / norm(n);
    t = cross(e, n);
    Bi = Oi + x*e + h_elbow*t;
    %% α (OA → AB)
    OA = OA_dir;
    AB = Bi - Oi;
    alpha(i) = atan2(norm(cross(OA,AB)), dot(OA,AB));
    ref = cross(OA, [0;1;0]);
if norm(ref) < 1e-6
        ref = cross(OA, [0;0;1]);  % fallback for leg 1 which aligns with Y cross
end
    ref = ref / norm(ref);
if dot(cross(OA,AB), ref) < 0
        alpha(i) = -alpha(i);
end
    %% β (AB → BC signed)
    BC = Ci - Bi;
    nB = cross(AB, BC);
if norm(nB) < 1e-12
        nB = cross(AB, [0;1;0]);
end
    nB = nB / norm(nB);
    beta(i) = atan2(dot(nB, cross(AB,BC)), dot(AB,BC));
    %% store geometry
    A(:,i) = Oi;
    B(:,i) = Bi;
    C(:,i) = Ci;
end
alpha_deg = rad2deg(alpha); %as claude pointed out, this may hide when things are breaking, but gotta do it
beta_deg  = rad2deg(beta);
end