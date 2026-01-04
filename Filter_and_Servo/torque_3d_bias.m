% Test for external disturbance for 1) of 5.2.1
close all;
% Parameter settings
A_x = 0.05; f_x = 0.2; B_x = 0.1;
A_y = 0.05; f_y = 0.4; B_y = 0.1;
A_z = 0; f_z = 0.5; B_z = 0.0;

% Time settings
t = linspace(0, 5, 1000);

% Three-axis disturbance torque = bias + sinusoidal terms
tau_x = B_x + A_x * sin(2 * pi * f_x * t);
tau_y = B_y + A_y * sin(2 * pi * f_y * t);
tau_z = B_z + A_z * sin(2 * pi * f_z * t);

% Plot 3D trajectory
figure;
plot3(tau_x, tau_y, tau_z, 'LineWidth', 1.5);
grid on;
xlabel('\tau_x (Nm)');
ylabel('\tau_y (Nm)');
zlabel('\tau_z (Nm)');
title('3D Sinusoidal Torque Disturbance with Bias');
% view(135, 30);
% axis equal;