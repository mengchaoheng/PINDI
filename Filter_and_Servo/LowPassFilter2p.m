clear;
close all;
clc;

% PX4 math::LowPassFilter2p numerical test.
% Current SITL mixer/update rate measured in 07_12_00.ulg is 250 Hz.
% Change this value if another log has a different measured update rate.
sample_freq = 250;
cutoff_list = [20, 10, 5, 3, 1,0.9,0.8,0.7,0.6, 0.5,0.4,0.3,0.2, 0.1, 0.05, 0.01]; % have problem in 0.2

% Long enough to observe the 0.01 Hz response.
dt = 1 / sample_freq;
t_end = 100;
t = (0:dt:t_end)';
u = zeros(size(t));
u(t >= 1) = 1;

N_cutoff = length(cutoff_list);
Y_single = zeros(length(t), N_cutoff);
Y_double = zeros(length(t), N_cutoff);
Y_px4_limited = zeros(length(t), N_cutoff);

fprintf('\nPX4 LowPassFilter2p numerical test\n');
fprintf('Sample frequency: %.3f Hz\n\n', sample_freq);
fprintf(['requested  PX4_effective  pole_single  DCgain_single  ' ...
         'max|single-double|  finite\n']);

for i = 1:N_cutoff
    fc = cutoff_list(i);

    % Requested cutoff without the PX4 5 Hz lower limit.
    c_single = coefficients_single(sample_freq, fc, false);
    c_double = coefficients_double(sample_freq, fc);

    Y_single(:, i) = filter_df2_single(u, c_single);
    Y_double(:, i) = filter_df2_double(u, c_double);

    % Current PX4 behavior: every positive cutoff below 5 Hz becomes 5 Hz.
    c_px4 = coefficients_single(sample_freq, fc, true);
    Y_px4_limited(:, i) = filter_df2_single(u, c_px4);

    poles_single = roots([1, double(c_single.a1), double(c_single.a2)]);
    pole_radius = max(abs(poles_single));

    denominator_single = single(1) + c_single.a1 + c_single.a2;
    numerator_single = c_single.b0 + c_single.b1 + c_single.b2;
    dc_gain_single = double(numerator_single / denominator_single);

    max_error = max(abs(Y_single(:, i) - Y_double(:, i)));
    is_finite = all(isfinite(Y_single(:, i)));

    fprintf('%9.3g  %13.3g  %11.9f  %13.7g  %18.7g  %d\n', ...
        fc, c_px4.fc, pole_radius, dc_gain_single, max_error, is_finite);
end

%% Figure 1: requested cutoff without the 5 Hz limit
figure('Name', 'LowPassFilter2p requested cutoffs', 'Color', 'w');
colors = lines(N_cutoff);

for i = 1:N_cutoff
    plot(t, Y_single(:, i), 'Color', colors(i, :), ...
        'LineWidth', 1, ...
        'DisplayName', sprintf('f_c = %.3g Hz', cutoff_list(i)));
    hold on;
end

grid on;
xlabel('Time (s)');
ylabel('Output');
title(sprintf('PX4 Direct Form II, single precision, f_s = %.0f Hz', sample_freq));
legend('Location', 'eastoutside');

%% Figure 2: original 5 Hz limit versus the requested low cutoff
test_fc = 0.3;
test_index = find(cutoff_list == test_fc, 1);

figure('Name', 'PX4 5 Hz lower-limit comparison', 'Color', 'w');
plot(t, u, 'k:', 'LineWidth', 0.8); hold on;
plot(t, Y_px4_limited(:, test_index), 'r-', 'LineWidth', 1.2);
plot(t, Y_single(:, test_index), 'b--', 'LineWidth', 1.2);
plot(t, Y_double(:, test_index), 'g-.', 'LineWidth', 1.0);
grid on;
xlabel('Time (s)');
ylabel('Output');
title(sprintf('Requested %.3g Hz: original PX4 path uses 5 Hz', test_fc));
legend('Input', 'PX4 limited: 5 Hz single', ...
       sprintf('Unclamped: %.3g Hz single', test_fc), ...
       sprintf('Unclamped: %.3g Hz double', test_fc), ...
       'Location', 'southeast');

%% Figure 3: single-precision numerical error
max_error = zeros(1, N_cutoff);
final_error = zeros(1, N_cutoff);

for i = 1:N_cutoff
    max_error(i) = max(abs(Y_single(:, i) - Y_double(:, i)));
    final_error(i) = abs(Y_single(end, i) - Y_double(end, i));
end

figure('Name', 'LowPassFilter2p numerical error', 'Color', 'w');
loglog(cutoff_list, max_error, 'bo-', 'LineWidth', 1); hold on;
loglog(cutoff_list, final_error, 'rs--', 'LineWidth', 1);
grid on;
xlabel('Cutoff frequency (Hz)');
ylabel('Absolute error');
title(sprintf('Single versus double precision, f_s = %.0f Hz', sample_freq));
legend('Maximum error', 'Final-value error', 'Location', 'best');

%% Figure 4 (new): random signal, like FirstOrderFilter.m
% Upper panel: directly see what increasingly low cutoff frequencies do.
% Lower panel: distinguish normal low-pass attenuation from numerical error.
rng(1);
random_fc_list = [1, 0.5, 0.3, 0.2, 0.1, 0.05, 0.01];
u_random = sin(2*pi*0.05*t) + 0.5*randn(size(t));
Y_random_single = zeros(length(t), length(random_fc_list));
Y_random_double = zeros(size(Y_random_single));
relative_rms_error = zeros(size(random_fc_list));

for i = 1:length(random_fc_list)
    c_single = coefficients_single(sample_freq, random_fc_list(i), false);
    c_double = coefficients_double(sample_freq, random_fc_list(i));
    Y_random_single(:, i) = filter_df2_single(u_random, c_single);
    Y_random_double(:, i) = filter_df2_double(u_random, c_double);

    error_signal = Y_random_single(:, i) - Y_random_double(:, i);
    rms_error = sqrt(mean(error_signal.^2));
    rms_reference = sqrt(mean(Y_random_double(:, i).^2));
    relative_rms_error(i) = 100*rms_error/max(rms_reference, eps);
end

figure('Name', 'Low cutoff random-signal test', 'Color', 'w');
tiledlayout(2, 1);

nexttile;
plot(t, u_random, 'Color', [0.75 0.75 0.75], 'LineWidth', 0.5, ...
    'DisplayName', 'Input: 0.05 Hz sine + random noise');
hold on;
random_colors = lines(length(random_fc_list));
for i = 1:length(random_fc_list)
    plot(t, Y_random_single(:, i), 'Color', random_colors(i, :), ...
        'LineWidth', 1, ...
        'DisplayName', sprintf('f_c = %.2g Hz', random_fc_list(i)));
end
grid on;
xlabel('Time (s)');
ylabel('Output');
title(sprintf('PX4 single-precision output, f_s = %.0f Hz', sample_freq));
legend('Location', 'eastoutside');

nexttile;
loglog(random_fc_list, relative_rms_error, 'bo-', ...
    'LineWidth', 1.2, 'MarkerFaceColor', 'b');
hold on;
yline(1, 'r--', '1% error');
grid on;
xlabel('Cutoff frequency (Hz)');
ylabel('Single/double relative RMS error (%)');
title('Numerical error (not normal filtering attenuation)');

fprintf('\nRandom-signal single/double precision comparison\n');
fprintf('cutoff(Hz)  relative RMS error(%%)\n');
for i = 1:length(random_fc_list)
    fprintf('%10.3g  %21.6g\n', random_fc_list(i), relative_rms_error(i));
end

%% PX4 single-precision coefficients
function c = coefficients_single(fs, fc, use_5hz_limit)
fs = single(fs);
fc_requested = single(fc);

if fs <= 0 || fc_requested <= 0 || fc_requested >= fs/2
    c.b0 = single(1);
    c.b1 = single(0);
    c.b2 = single(0);
    c.a1 = single(0);
    c.a2 = single(0);
    c.fc = 0;
    return;
end

if use_5hz_limit
    fc_used = max(fc_requested, single(5));
else
    fc_used = fc_requested;
end

fr = fs / fc_used;
ohm = tan(single(pi) / fr);
normalizer = single(1) ...
    + single(2)*cos(single(pi)/single(4))*ohm ...
    + ohm*ohm;

c.b0 = ohm*ohm / normalizer;
c.b1 = single(2)*c.b0;
c.b2 = c.b0;
c.a1 = single(2)*(ohm*ohm - single(1)) / normalizer;
c.a2 = (single(1) ...
    - single(2)*cos(single(pi)/single(4))*ohm ...
    + ohm*ohm) / normalizer;
c.fc = double(fc_used);
end

%% Double-precision reference coefficients, no 5 Hz limit
function c = coefficients_double(fs, fc)
ohm = tan(pi * fc / fs);
normalizer = 1 + sqrt(2)*ohm + ohm^2;

c.b0 = ohm^2 / normalizer;
c.b1 = 2*c.b0;
c.b2 = c.b0;
c.a1 = 2*(ohm^2 - 1) / normalizer;
c.a2 = (1 - sqrt(2)*ohm + ohm^2) / normalizer;
end

%% Exact PX4 Direct Form II using single-precision state
function y = filter_df2_single(u, c)
u = single(u);
y = zeros(size(u), 'single');
delay1 = single(0);
delay2 = single(0);

for k = 1:length(u)
    delay0 = u(k) - delay1*c.a1 - delay2*c.a2;
    y(k) = delay0*c.b0 + delay1*c.b1 + delay2*c.b2;
    delay2 = delay1;
    delay1 = delay0;
end

y = double(y);
end

%% Double-precision Direct Form II reference
function y = filter_df2_double(u, c)
y = zeros(size(u));
delay1 = 0;
delay2 = 0;

for k = 1:length(u)
    delay0 = u(k) - delay1*c.a1 - delay2*c.a2;
    y(k) = delay0*c.b0 + delay1*c.b1 + delay2*c.b2;
    delay2 = delay1;
    delay1 = delay0;
end
end
