%% --- TO BE PAIRED WITH IK_3RRS_CHAT ---

close all;
clear all;
clc;


%Links in mm

Rb = 45.44;
Rp = 70.6;
L1 = 51;
L2 = 72.61;

%% ---------------- INPUT ----------------
%psi change
H = 90;
psi_n = [-15 -10 -5 0 5 10 15 10 5 0 -5 -10 -15];
theta = 0;

for h_i=1:length(psi_n)
    psi = psi_n(h_i);
%% ---------------- IK CALL ----------------
[alpha, beta, A, B, C, O, P] = IK_3RRS(Rb,Rp,L1,L2,H, psi, theta);

%% ---------------- PLOT ----------------
figure(1);
clf;
hold on; grid on; axis equal;
camproj('orthographic')

camtarget([0 0 0])
campos([200 200 200])
camup([0 1 0])

colors = ['r','b','g'];

%% ---------------- BASE ----------------
plot3([O(1,:) O(1,1)], ...
      [O(2,:) O(2,1)], ...
      [O(3,:) O(3,1)], 'k','LineWidth',2);

scatter3(O(1,:),O(2,:),O(3,:),50,'k','filled');

%% ---------------- PLATFORM ----------------
plot3([P(1,:) P(1,1)], ...
      [P(2,:) P(2,1)], ...
      [P(3,:) P(3,1)], 'm','LineWidth',2);

scatter3(P(1,:),P(2,:),P(3,:),50,'m','filled');

%% ---------------- LINKS ----------------
for i = 1:3

    plot3([A(1,i) B(1,i)], ...
          [A(2,i) B(2,i)], ...
          [A(3,i) B(3,i)], colors(i), 'LineWidth', 2);

    plot3([B(1,i) C(1,i)], ...
          [B(2,i) C(2,i)], ...
          [B(3,i) C(3,i)], colors(i), 'LineWidth', 2);

    scatter3(A(1,i),A(2,i),A(3,i),50,'k','filled');
    scatter3(B(1,i),B(2,i),B(3,i),50,colors(i),'filled');
    scatter3(C(1,i),C(2,i),C(3,i),50,'m','filled');

end

%% ---------------- LABELS ----------------
xlabel('X');
ylabel('Y');
zlabel('Z');
ylim([-10 150]);
xlim([-150 150]);
zlim([-100 100]);
set(gcf,'WindowState','maximized');
pause(0.5);
end

figure(1);
clf;
%H change

%% ---------------- INPUT ----------------
H_n = [120 110 100 90 80 70 80 90 100 110 120];
psi = 0;
theta = 0;

for h_i=1:length(H_n)
    H = H_n(h_i);
%% ---------------- IK CALL ----------------
[alpha, beta, A, B, C, O, P] = IK_3RRS(Rb,Rp,L1,L2,H, psi, theta);

%% ---------------- PLOT ----------------
figure(1);
clf;
hold on; grid on; axis equal;
camproj('orthographic')

camtarget([0 0 0])
campos([200 200 200])
camup([0 1 0])

colors = ['r','b','g'];

%% ---------------- BASE ----------------
plot3([O(1,:) O(1,1)], ...
      [O(2,:) O(2,1)], ...
      [O(3,:) O(3,1)], 'k','LineWidth',2);

scatter3(O(1,:),O(2,:),O(3,:),50,'k','filled');

%% ---------------- PLATFORM ----------------
plot3([P(1,:) P(1,1)], ...
      [P(2,:) P(2,1)], ...
      [P(3,:) P(3,1)], 'm','LineWidth',2);

scatter3(P(1,:),P(2,:),P(3,:),50,'m','filled');

%% ---------------- LINKS ----------------
for i = 1:3

    plot3([A(1,i) B(1,i)], ...
          [A(2,i) B(2,i)], ...
          [A(3,i) B(3,i)], colors(i), 'LineWidth', 2);

    plot3([B(1,i) C(1,i)], ...
          [B(2,i) C(2,i)], ...
          [B(3,i) C(3,i)], colors(i), 'LineWidth', 2);

    scatter3(A(1,i),A(2,i),A(3,i),50,'k','filled');
    scatter3(B(1,i),B(2,i),B(3,i),50,colors(i),'filled');
    scatter3(C(1,i),C(2,i),C(3,i),50,'m','filled');

end

%% ---------------- LABELS ----------------
xlabel('X');
ylabel('Y');
zlabel('Z');
ylim([-10 150]);
xlim([-150 150]);
zlim([-100 100]);
set(gcf,'WindowState','maximized');
pause(0.5);
end

figure(1);
clf;

%theta change
H = 90;
psi = 0;
theta_n = [-15 -10 -5 0 5 10 15 10 5 0 -5 -10 -15];

for h_i=1:length(theta_n)
    theta = theta_n(h_i);
%% ---------------- IK CALL ----------------
[alpha, beta, A, B, C, O, P] = IK_3RRS(Rb,Rp,L1,L2,H, psi, theta);

%% ---------------- PLOT ----------------
figure(1);
clf;
hold on; grid on; axis equal;
camproj('orthographic')

camtarget([0 0 0])
campos([200 200 200])
camup([0 1 0])

colors = ['r','b','g'];

%% ---------------- BASE ----------------
plot3([O(1,:) O(1,1)], ...
      [O(2,:) O(2,1)], ...
      [O(3,:) O(3,1)], 'k','LineWidth',2);

scatter3(O(1,:),O(2,:),O(3,:),50,'k','filled');

%% ---------------- PLATFORM ----------------
plot3([P(1,:) P(1,1)], ...
      [P(2,:) P(2,1)], ...
      [P(3,:) P(3,1)], 'm','LineWidth',2);

scatter3(P(1,:),P(2,:),P(3,:),50,'m','filled');

%% ---------------- LINKS ----------------
for i = 1:3

    plot3([A(1,i) B(1,i)], ...
          [A(2,i) B(2,i)], ...
          [A(3,i) B(3,i)], colors(i), 'LineWidth', 2);

    plot3([B(1,i) C(1,i)], ...
          [B(2,i) C(2,i)], ...
          [B(3,i) C(3,i)], colors(i), 'LineWidth', 2);

    scatter3(A(1,i),A(2,i),A(3,i),50,'k','filled');
    scatter3(B(1,i),B(2,i),B(3,i),50,colors(i),'filled');
    scatter3(C(1,i),C(2,i),C(3,i),50,'m','filled');

end

%% ---------------- LABELS ----------------
xlabel('X');
ylabel('Y');
zlabel('Z');
ylim([-10 150]);
xlim([-150 150]);
zlim([-100 100]);
set(gcf,'WindowState','maximized');
pause(0.5);
end