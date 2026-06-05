%% --- Parameters ---
Rb = 45.44; Rp = 70.6; L1 = 51; L2 = 72.61;

%% --- 1. Generate trajectory via direct IK sampling ---
n_cycles = 2;
N_servo  = 60 * n_cycles;
t_servo  = linspace(0, 2*pi * n_cycles, N_servo);

psi_amp   = 20;  % degrees
theta_amp = 0;   % degrees
H = 70;
H_amp         = 20;

alpha_servo = zeros(N_servo, 3);

for k = 1:N_servo
    psi_k   = psi_amp   * sin(t_servo(k));
    theta_k = theta_amp * sin(t_servo(k));
    H_k = H + H_amp * sin(t_servo(k));
    [alpha_deg, ~,~,~,~,~,~] = IK_3RRS_chat(Rb, Rp, L1, L2, H_k, psi_k, theta_k);
    alpha_servo(k, :) = alpha_deg;
end

%% --- 2. Plot ---
figure; hold on; grid on;
colors = ['r','b','g'];
leg_names = {'Leg 1','Leg 2','Leg 3'};

for leg = 1:3
    plot(t_servo, alpha_servo(:,leg), colors(leg), 'LineWidth', 1.5, ...
         'DisplayName', leg_names{leg});
    scatter(t_servo, alpha_servo(:,leg), 20, colors(leg), 'filled', ...
            'HandleVisibility', 'off');
end

xlabel('t (rad)'); ylabel('alpha (deg)');
title('Direct IK Sampling — Servo Waypoints');
legend show;

%% --- 3. Return to neutral ---
neutral_psi   = 0;
neutral_theta = 0;
neutral_H     = 90;

[alpha_neutral, ~,~,~,~,~,~] = IK_3RRS_chat(Rb, Rp, L1, L2, ...
                                 neutral_H, neutral_psi, neutral_theta);
alpha_end = alpha_servo(end, :);
N_return  = 20;
alpha_return = zeros(N_return, 3);

for leg = 1:3
    alpha_return(:, leg) = linspace(alpha_end(leg), alpha_neutral(leg), N_return);
end

%% --- 4. Build full trajectory ---
alpha_full = [alpha_servo; alpha_return];

%% --- 5. Print waypoints ---
disp('Servo waypoints (degrees) — columns are legs 1, 2, 3:');
disp(alpha_servo);

fprintf('\nReturn to neutral waypoints (degrees) — columns are legs 1, 2, 3:\n');
fprintf('%8s  %8s  %8s\n', 'Leg 1', 'Leg 2', 'Leg 3');
for k = 1:N_return
    fprintf('%8.2f  %8.2f  %8.2f\n', alpha_return(k,1), alpha_return(k,2), alpha_return(k,3));
end

%% --- 6. Send over serial ---
s = serialport("COM5", 115200);
s.Timeout = 10;
configureTerminator(s, "LF");

fprintf('Waiting for Arduino...\n');
response = readline(s);
if strlength(strtrim(response)) > 0
    fprintf('Arduino ready: %s\n', response);
else
    fprintf('No response from Arduino — check Serial.begin() in sketch\n');
    return;
end

safe = true;

for k = 1:size(alpha_full, 1)
    msg = sprintf('%.2f,%.2f,%.2f\n', alpha_full(k,1), alpha_full(k,2), alpha_full(k,3));
    write(s, msg, 'char');
    response = readline(s);

    if ~isstring(response) && ~ischar(response)
        fprintf('Bad response at step %d, aborting.\n', k);
        safe = false;
        break;
    elseif strlength(strtrim(response)) == 0
        fprintf('Empty response at step %d — Arduino may not be responding\n', k);
        safe = false;
        break;
    elseif startsWith(response, 'OUT_OF_RANGE')
        safe = false;
        fprintf('Out of range at step %d: %s\n', k, response);
        fprintf('Generating emergency return to neutral...\n');

        vals = str2double(split(extractAfter(response, 'OUT_OF_RANGE:'), ','));
        alpha_bad = vals';

        [alpha_neutral, ~,~,~,~,~,~] = IK_3RRS_chat(Rb, Rp, L1, L2, 90, 0, 0);
        N_emergency = 20;
        alpha_emergency = zeros(N_emergency, 3);
        for leg = 1:3
            alpha_emergency(:,leg) = linspace(alpha_bad(leg), alpha_neutral(leg), N_emergency);
        end

        fprintf('Sending emergency return...\n');
        for j = 1:N_emergency
            emsg = sprintf('%.2f,%.2f,%.2f\n', alpha_emergency(j,1), alpha_emergency(j,2), alpha_emergency(j,3));
            write(s, emsg, 'char');
            ack = readline(s);
            pause(0.025);
        end

        fprintf('Returned to neutral. Stopping trajectory.\n');
        break;

    else
        pause(0.025);
    end
end

if safe
    fprintf('Trajectory completed successfully.\n');
else
    fprintf('Trajectory aborted — check manipulator before continuing.\n');
end