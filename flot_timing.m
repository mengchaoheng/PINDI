clear;
close all;
clc;

% Current fixed-chain verification. Do not add .ulg.
% Use log_53 only when a historical before/after comparison is needed.
ulgFileName = 'data/log_54_2026-7-25-15-48-54';
pwm_main_rate = 333; % Physical PWM frequency (Hz)

rootDir = fileparts(mfilename('fullpath'));
ulgFile = fullfile(rootDir, [ulgFileName '.ulg']);
[ulgDir, logName] = fileparts(ulgFile);
csvFile = fullfile(ulgDir, [logName '_allocation_value_0.csv']);

%% Convert only allocation_value
if ismac
    ulog2csv = fullfile(getenv('HOME'), ...
        'Library/Python/3.9/bin/ulog2csv');
else
    ulog2csv = 'ulog2csv';
end

command = sprintf('"%s" -m allocation_value -o "%s" "%s"', ...
    ulog2csv, ulgDir, ulgFile);

if system(command) ~= 0
    error('ulog2csv failed: %s', ulgFile);
end

data = readtable(csvFile);
delete(csvFile);

if ~ismember('mixer_update_count', data.Properties.VariableNames)
    error('The log does not contain the new timing fields.');
end

%% Calculate actual periods
timestamp = double(data.timestamp);
t = (timestamp - timestamp(1)) * 1e-6;
log_dt = diff(timestamp) * 1e-6;

mixer_count = double(data.mixer_update_count);
control_count = double(data.control_generation);
long_count = double(data.mixer_long_dt_count);
change_count = double(data.sample_freq_change_count);
reset_count = double(data.filter_reset_count);

dt_window = diff(timestamp) * 1e-6;
t_mid = (t(1:end-1) + t(2:end)) / 2;
mixer_hz_window = diff(mixer_count) ./ dt_window;
control_hz_window = diff(control_count) ./ dt_window;

total_time = (timestamp(end) - timestamp(1)) * 1e-6;
mixer_hz = (mixer_count(end) - mixer_count(1)) / total_time;
control_hz = (control_count(end) - control_count(1)) / total_time;
logger_hz_mean = (height(data) - 1) / total_time;
logger_hz_local = 1 / median(log_dt);

long_ratio = 100 * (long_count(end) - long_count(1)) ...
    / (mixer_count(end) - mixer_count(1));
reset_ratio = 100 * (reset_count(end) - reset_count(1)) ...
    / (mixer_count(end) - mixer_count(1));
reset_rate = (reset_count(end) - reset_count(1)) / total_time;
mixer_dt = double(data.mixer_dt_us);
short_dt = mixer_dt(mixer_dt < 2000);
long_dt = mixer_dt(mixer_dt >= 2000);

fprintf('\n========== Actual timing ==========\n');
fprintf('Physical PWM:      %.3f Hz, %.3f ms\n', ...
    pwm_main_rate, 1000 / pwm_main_rate);
fprintf('Controller:        %.3f Hz, %.3f ms\n', ...
    control_hz, 1000 / control_hz);
fprintf('Mixer mean:        %.3f Hz, %.3f ms\n', ...
    mixer_hz, 1000 / mixer_hz);
fprintf('ULog mean rate:    %.3f Hz\n', logger_hz_mean);
fprintf('ULog local median: %.3f Hz, %.3f ms\n', ...
    logger_hz_local, 1000 / logger_hz_local);
fprintf('Topic interval:    %.0f us\n', ...
    median(double(data.topic_update_interval_us)));
fprintf('Mixer dt sampled:  min %.0f, median %.0f, max %.0f us\n', ...
    min(mixer_dt), median(mixer_dt), max(mixer_dt));
fprintf('Short mixer dt:    %.3f ms, %.2f %%\n', ...
    mean(short_dt) / 1000, 100 - long_ratio);
fprintf('Long mixer dt:     %.3f ms, %.2f %%\n', ...
    mean(long_dt) / 1000, long_ratio);
fprintf('Frequency changes: %d\n', ...
    round(change_count(end) - change_count(1)));
fprintf('Filter resets:     %d, %.2f %% cycles, %.2f /s\n', ...
    round(reset_count(end) - reset_count(1)), ...
    reset_ratio, reset_rate);
fprintf('Filter cutoff:     %.3f Hz\n', ...
    median(double(data.filter_cutoff_hz)));
fprintf('===================================\n\n');

%% Plot
figure('Name', 'PX4 actual timing', 'Color', 'w');

subplot(3, 1, 1);
plot(t, double(data.mixer_dt_us) / 1000, '.');
grid on;
ylabel('Mixer dt (ms)');
title('Mixer instantaneous period');

subplot(3, 1, 2);
plot(t_mid, control_hz_window, 'b-'); hold on;
plot(t_mid, mixer_hz_window, 'r-');
yline(2 * pwm_main_rate, 'k--');
grid on;
ylabel('Frequency (Hz)');
legend('Controller', 'Mixer', '2 x PWM', 'Location', 'best');

subplot(3, 1, 3);
stairs(t, long_count - long_count(1), 'k-'); hold on;
stairs(t, change_count - change_count(1), 'b-');
stairs(t, reset_count - reset_count(1), 'r-');
grid on;
xlabel('Time (s)');
ylabel('Count');
legend('dt >= 2 ms', 'frequency change', ...
    'filter reset', 'Location', 'best');
