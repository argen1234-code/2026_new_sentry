clear; clc; close all;

% Fit yaw inertia from VOFA csv exported by current gimbal_task.c.
% VOFA channels:
% I0: yaw reference angle [rad]
% I1: yaw actual angle [rad]
% I2: yaw MIT feedback velocity [rad/s]
% I3: yaw MIT feedback acceleration from firmware diff(velocity) [rad/s^2]
% I4: yaw MIT feedback torque
% I5: yaw final command torque

csv_file = 'vofa+.csv';
dt = 0.001;              % GIMBAL_CONTROL_TIME = 1 ms. Change if firmware period changes.
main_window_ms = 31;     % Recommended smoothing window for velocity before differentiation.
window_ms_list = [5 9 15 21 31 41 61 81 101];
min_abs_accel = 1.0;     % Ignore near-zero acceleration points.

data = readtable(csv_file);
ref = data.I0;
pos = data.I1;
vel = data.I2;
acc_raw = data.I3;
torque_fb = data.I4;
torque_cmd = data.I5;

n = numel(ref);
t = (0:n-1)' * dt;

fprintf('Loaded %s: %d samples, %.3f s\n', csv_file, n, t(end));
fprintf('Command saturation ratio |I5| >= 0.999: %.2f %%\n', 100 * mean(abs(torque_cmd) >= 0.999));
fprintf('Feedback torque saturation ratio |I4| >= 0.95: %.2f %%\n', 100 * mean(abs(torque_fb) >= 0.95));
fprintf('Tracking RMS error: %.6f rad, max error: %.6f rad\n\n', sqrt(mean((ref - pos).^2)), max(abs(ref - pos)));

% Direct firmware acceleration fit. This is useful as a baseline, but it is
% usually noisy because MIT velocity quantization is differentiated at 1 ms.
mask_raw = isfinite(acc_raw) & isfinite(vel) & isfinite(torque_fb) & abs(acc_raw) > min_abs_accel;
[beta_raw, stat_raw] = fit_inertia_model(acc_raw, vel, torque_fb, mask_raw);
print_fit('Raw I3 acceleration', beta_raw, stat_raw);

% Recommended fit: smooth feedback velocity first, then differentiate offline.
main_win = ms_to_odd_samples(main_window_ms, dt);
[vel_smooth, acc_smooth] = smooth_velocity_and_diff(vel, main_win, dt);
edge = main_win;
mask_main = true(n, 1);
mask_main(1:edge) = false;
mask_main(end-edge+1:end) = false;
mask_main = mask_main & isfinite(acc_smooth) & isfinite(vel_smooth) & isfinite(torque_fb) ...
            & abs(acc_smooth) > min_abs_accel;

[beta_main, stat_main, torque_fit] = fit_inertia_model(acc_smooth, vel_smooth, torque_fb, mask_main);
print_fit(sprintf('Smoothed velocity derivative, window = %d ms', main_window_ms), beta_main, stat_main);

% Sweep smoothing windows to see whether J is stable.
J_list = zeros(size(window_ms_list));
R2_list = zeros(size(window_ms_list));
RMSE_list = zeros(size(window_ms_list));
for k = 1:numel(window_ms_list)
    win = ms_to_odd_samples(window_ms_list(k), dt);
    [v_s, a_s] = smooth_velocity_and_diff(vel, win, dt);
    m = true(n, 1);
    m(1:win) = false;
    m(end-win+1:end) = false;
    m = m & isfinite(a_s) & isfinite(v_s) & isfinite(torque_fb) & abs(a_s) > min_abs_accel;
    [b, st] = fit_inertia_model(a_s, v_s, torque_fb, m);
    J_list(k) = b(1);
    R2_list(k) = st.r2;
    RMSE_list(k) = st.rmse;
end

fprintf('Window sweep:\n');
fprintf('  window_ms        J             R2          RMSE\n');
for k = 1:numel(window_ms_list)
    fprintf('  %8d   % .8f   %.5f   %.5f\n', window_ms_list(k), J_list(k), R2_list(k), RMSE_list(k));
end

fprintf('\nRecommended yaw inertia J = %.6f kg*m^2\n', beta_main(1));
fprintf('Use the window sweep as confidence check. Current stable range is roughly %.6f ~ %.6f kg*m^2.\n', ...
        min(J_list(window_ms_list >= 15 & window_ms_list <= 61)), ...
        max(J_list(window_ms_list >= 15 & window_ms_list <= 61)));

% Plot 1: reference tracking and velocity/acceleration.
figure('Name', 'Yaw VOFA Data Overview', 'Color', 'w');
tiledlayout(3, 1, 'TileSpacing', 'compact');

nexttile;
plot(t, ref, 'LineWidth', 1.0); hold on;
plot(t, pos, 'LineWidth', 1.0);
grid on;
xlabel('Time [s]');
ylabel('Angle [rad]');
legend('Reference I0', 'Actual I1', 'Location', 'best');
title('Yaw angle tracking');

nexttile;
plot(t, vel, 'Color', [0.65 0.65 0.65]); hold on;
plot(t, vel_smooth, 'LineWidth', 1.0);
grid on;
xlabel('Time [s]');
ylabel('Velocity [rad/s]');
legend('Raw I2', sprintf('Smoothed I2, %d ms', main_window_ms), 'Location', 'best');
title('Yaw feedback velocity');

nexttile;
plot(t, acc_raw, 'Color', [0.75 0.75 0.75]); hold on;
plot(t, acc_smooth, 'LineWidth', 1.0);
grid on;
xlabel('Time [s]');
ylabel('Acceleration [rad/s^2]');
legend('Raw I3', 'Offline acceleration', 'Location', 'best');
title('Acceleration used for fitting');

% Plot 2: torque fitting.
figure('Name', 'Yaw Inertia Fit', 'Color', 'w');
tiledlayout(3, 1, 'TileSpacing', 'compact');

nexttile;
plot(t, torque_fb, 'Color', [0.2 0.2 0.2]); hold on;
plot(t(mask_main), torque_fit, '.', 'MarkerSize', 4);
grid on;
xlabel('Time [s]');
ylabel('Torque');
legend('Feedback torque I4', 'Fitted torque', 'Location', 'best');
title(sprintf('Torque fit: J=%.6f, B=%.6f, Fc=%.6f, bias=%.6f, R^2=%.4f', ...
      beta_main(1), beta_main(2), beta_main(3), beta_main(4), stat_main.r2));

nexttile;
torque_friction_removed = torque_fb - beta_main(2) * vel_smooth ...
                         - beta_main(3) * sign(vel_smooth) - beta_main(4);
scatter(acc_smooth(mask_main), torque_friction_removed(mask_main), 5, '.');
hold on;
a_line = linspace(min(acc_smooth(mask_main)), max(acc_smooth(mask_main)), 200)';
plot(a_line, beta_main(1) * a_line, 'r', 'LineWidth', 1.5);
grid on;
xlabel('Acceleration [rad/s^2]');
ylabel('Torque after friction/bias removal');
title('Inertia slope check');

nexttile;
yyaxis left;
plot(window_ms_list, J_list, '-o', 'LineWidth', 1.2);
ylabel('J [kg*m^2]');
yyaxis right;
plot(window_ms_list, R2_list, '-s', 'LineWidth', 1.2);
ylabel('R^2');
grid on;
xlabel('Velocity smoothing window [ms]');
title('J stability versus smoothing window');

function [beta, stat, y_fit] = fit_inertia_model(accel, velocity, torque, mask)
    % Model: torque = J * accel + B * velocity + Fc * sign(velocity) + bias
    x = [accel(mask), velocity(mask), sign(velocity(mask)), ones(nnz(mask), 1)];
    y = torque(mask);
    beta = x \ y;
    y_fit = x * beta;
    residual = y - y_fit;
    stat.rmse = sqrt(mean(residual.^2));
    stat.r2 = 1 - sum(residual.^2) / sum((y - mean(y)).^2);
    stat.n = nnz(mask);
end

function print_fit(name, beta, stat)
    fprintf('%s:\n', name);
    fprintf('  n    = %d\n', stat.n);
    fprintf('  J    = %.8f kg*m^2\n', beta(1));
    fprintf('  B    = %.8f\n', beta(2));
    fprintf('  Fc   = %.8f\n', beta(3));
    fprintf('  bias = %.8f\n', beta(4));
    fprintf('  R2   = %.5f\n', stat.r2);
    fprintf('  RMSE = %.5f\n\n', stat.rmse);
end

function samples = ms_to_odd_samples(window_ms, dt)
    samples = max(3, round((window_ms * 0.001) / dt));
    if mod(samples, 2) == 0
        samples = samples + 1;
    end
end

function [vel_smooth, accel] = smooth_velocity_and_diff(velocity, win, dt)
    vel_smooth = movmean(velocity, win, 'Endpoints', 'shrink');
    accel = gradient(vel_smooth, dt);
end
