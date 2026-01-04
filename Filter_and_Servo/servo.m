close all;
% First-order system parameters

% dt = 1/500;    % Sampling period
% T = 15*dt;      % Time constant 0.03
dt = 1/250    % Sampling period
T = 0.03      % Time constant 0.03 > dt
N = 1000;      % Total number of samples
f = 0.05;      % Sine wave frequency (Hz)

% Time and input signal (sine wave)
t = (0:N-1) * dt;
% u = sin(2 * pi * f * t);   % Sine input
u = 3*0.3491*ones(1, N);            % Unit step
u(1:100) = 0;              % First 100 samples are 0, then 1
% Initialize output signal
y = zeros(1, N);

% Simulate real-time system
for k = 1:N
    y(k) = first_order_update(u(k), T, dt);
end

% Plot
figure;
plot(t, u, 'b--', 'LineWidth', 1.2); hold on;
plot(t, y, 'r-',  'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Signal');
legend('Input u(k)', 'Output y(k)');
title('First-Order System Response: Sine Input');
grid on;

% Real-time first-order system function
function y = first_order_update(u, T, dt)
    persistent y_prev
    if isempty(y_prev)
        y_prev = 0;
    end
    y = y_prev + (dt / T) * (u - y_prev);
    y_prev = y;
end