close all;
% Simulation parameters
T = 0.03;             % Time constant
dt = 1/250;           % Sampling period
t_end = 0.5;         % Simulation duration
t = 0:dt:t_end;      % Discrete time axis
N = length(t);

% Amplitude
Amp=20*pi/180;
% Input signal: unit step
u = Amp*ones(1, N);

% Initialize outputs for each method
y_true = Amp*(1 - exp(-t / T));   % Continuous-time analytical solution
y_zoh = zeros(1, N);
y_fwd = zeros(1, N);
y_bwd = zeros(1, N);
y_tustin = zeros(1, N);

% ===== Calculate difference coefficients for each method =====

% 1. ZOH
Az = exp(-dt / T);
Bz = 1 - Az;

% 2. Euler Forward
Af = 1 - dt / T;
Bf = dt / T;

% 3. Euler Backward
Ab = T / (T + dt);
Bb = dt / (T + dt);

% 4. Tustin（双线性变换）
alpha = (2*T - dt) / (2*T + dt);
beta = dt / (2*T + dt);

% ===== Recursive calculation =====
for k = 2:N
    y_zoh(k)     = Az * y_zoh(k-1) + Bz * u(k-1);
    y_fwd(k)     = Af * y_fwd(k-1) + Bf * u(k-1);
    y_bwd(k)     = Ab * y_bwd(k-1) + Bb * u(k);
    y_tustin(k)  = alpha * y_tustin(k-1) + beta * (u(k) + u(k-1));
end

% ===== Plot =====
figure;
plot(t, y_true, 'k:', 'LineWidth', 1); hold on;
plot(t, y_zoh, 'b--', 'LineWidth', 1);
plot(t, y_fwd, 'r-.', 'LineWidth', 1);
plot(t, y_bwd, 'g--', 'LineWidth', 1);
% plot(t, y_tustin, 'm:', 'LineWidth', 1.8);
legend('True Continuous', 'ZOH', 'Euler Forward', 'Euler Backward', 'Tustin');
xlabel('Time (s)');
ylabel('Output y(t)');
axis([0 t_end 0 1.2*Amp]);
title('Step Response Comparison of Discretization Methods');
grid on;


y_zoh(2)
y_fwd(2)
y_bwd(2) 
y_tustin(2)