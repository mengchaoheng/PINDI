% ===== Parameter settings =====
dt = 0.001;       % Sampling period (seconds)
t_end = 2;        % Total simulation time (seconds)
t = 0:dt:t_end;   % Time axis
N = length(t);

% ===== Input signal: 10Hz sine + white noise =====
f_input = 10;  % Input frequency (Hz)
u = sin(2*pi*f_input*t) + 0.5 * randn(1, N);  % Sine + noise

% ===== Different cutoff frequencies fc correspond to different time constants T =====
fc_list = [1, 3, 10, 30];         % Cutoff frequencies (Hz)
T_list = 1 ./ (2 * pi * fc_list); % T = 1/(2πfc)

% ===== Initialize results =====
Y = zeros(length(fc_list), N);   % Each row is the output of one filter

% ===== Apply filter for each cutoff frequency =====
for i = 1:length(T_list)
    T = T_list(i);
    y = zeros(1, N);
    for k = 2:N
        y(k) = y(k-1) + (dt / T) * (u(k) - y(k-1));  % Euler first-order low-pass filter
    end
    Y(i, :) = y;
end

% ===== Plot comparison =====
figure;
plot(t, u, 'k:', 'DisplayName', 'Original Input u(t)'); hold on;
colors = lines(length(fc_list));
for i = 1:length(fc_list)
    plot(t, Y(i, :), 'Color', colors(i,:), 'LineWidth', 1.5, ...
         'DisplayName', sprintf('f_c = %.1f Hz', fc_list(i)));
end
xlabel('Time (s)');
ylabel('Signal Amplitude');
title('Response of First-Order Low-Pass Filter at Different Cutoff Frequencies');
legend('show');
grid on;