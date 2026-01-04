close all;
% Parameter settings
A = 2;          % Amplitude (N·m)
f = 0.5;        % Frequency (Hz)
t = 0:0.01:10;  % Time range: 0~10 seconds, step size 0.01 seconds

% Calculate sinusoidal disturbance
tau = A * sin(2 * pi * f * t);

% Plot
figure;
plot(t, tau, 'b-', 'LineWidth', 2);   % Blue solid line, line width 2
hold on;
yline(0, 'k--');                      % Add black dashed line for zero reference
xlabel('Time t (s)');
ylabel('Disturbance Torque \tau(t) (N·m)');
title('Sinusoidal Disturbance Waveform: \tau(t) = A \cdot \sin(2\pi f t)');
legend('Disturbance Torque', 'Zero Reference Line', 'Location', 'southeast');
grid on;
axis([0 10 -A*1.2 A*1.2]);            % Set axis limits