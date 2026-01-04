% Basic parameters
dt = 0.001;      % Sampling period
t_end = 2;       % Total time
t = 0:dt:t_end;
N = length(t);

% Input signal: 10Hz sine + noise
f_input = 10;
u = sin(2 * pi * f_input * t) + 0.5 * randn(size(t));

% Set different cutoff frequencies (corresponding to different T)
fc_list = [1, 5, 10, 20];  % Hz
T_list = 1 ./ (2 * pi * fc_list);  % T = 1/(2*pi*fc)

% Initialize output matrix
Y = zeros(length(fc_list), N);

% Apply filter for each cutoff frequency
for i = 1:length(T_list)
    T = T_list(i);
    y = zeros(1, N);
    for k = 2:N
        y(k) = y(k-1) + (dt / T) * (u(k) - y(k-1));
    end
    Y(i, :) = y;
end

% Plot
figure;
plot(t, u, 'k--', 'DisplayName', 'Input Signal'); hold on;
colors = 'rbcm';
for i = 1:length(fc_list)
    plot(t, Y(i, :), colors(i), 'DisplayName', sprintf('f_c = %.1f Hz', fc_list(i)));
end
xlabel('Time (s)');
ylabel('Signal Amplitude');
title('Filtering Effect of Different Cutoff Frequencies on Input Signal');
legend;
grid on;