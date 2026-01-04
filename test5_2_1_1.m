clear all;
close all;
clc;
addpath(genpath(pwd));
% you can run on terminal 
% ulog2csv log_.ulg 
% to get csv files
% =====================1==========================
% Install pyulog using pip first.https://github.com/PX4/pyulog.
% in MacOS, it maybe have been installed by the px4-dev
% =====================2==========================
% Make sure it has installed ulog2csv correctly (check the output of which ulog2csv in Linux/MacOS or where ulog2csv in Windows).
% =====================3==========================
% Change the following line in ulogviewver.m:
% command = ['!/usr/local/bin/ulog2csv ' ulgFileName '.ulg'];
% to 
% command = ['!your ulog2csv path' ulgFileName '.ulg'];
% and 
% ulgFileName = '00_41_22'; 
% to 
% ulgFileName = 'your log name (full path)'; 

% ----fig size, you have to change it for your fig

d2r=pi/180;
r2d=180/pi;
%------------------------------------------
% Set ULog relative path
%------------------------------------------
ulgFileName = 'data/01_43_51';
tmp = [ulgFileName '.mat'];

% Record the current main script path
rootDir = fileparts(mfilename('fullpath'));

%------------------------------------------
% Step 1: Check if MAT file already exists
%------------------------------------------
if exist(fullfile(rootDir, tmp), "file")
    disp(['Found MAT file: ' tmp]);
    load(fullfile(rootDir, tmp), 'log');

else
    disp('No MAT file found, start parsing ULog...');

    %------------------------------------------
    % Step 2: Run ulog2csv (keep full path)
    %------------------------------------------
    if ismac
        ulog2csv_path = '~/Library/Python/3.9/bin/ulog2csv';
    else
        ulog2csv_path = 'ulog2csv';
    end

    ulgAbs = fullfile(rootDir, [ulgFileName '.ulg']);
    command = ['!' ulog2csv_path ' ' '"' ulgAbs '"'];
    disp(['Running command: ' command]);
    eval(command);

    %------------------------------------------
    % Step 3: Call parsing function (pass full path)
    %------------------------------------------
    log.data = csv_topics_to_d(fullfile(rootDir, ulgFileName));
    log.FileName = ulgFileName;
    log.version = 1.0;
    log.params = '';
    log.messages = '';
    log.info = '';

    %------------------------------------------
    % Step 4: Save MAT file to the same directory
    %------------------------------------------
    save(fullfile(rootDir, tmp), 'log');
    disp(['Saved MAT file: ' tmp]);

    %------------------------------------------
    % Step 5: Delete temporary CSV files
    %------------------------------------------
    delete(fullfile(rootDir, [ulgFileName '_*.csv']));
    disp('Temporary CSV files deleted.');
end
%%

if(isfield(log.data, 'parameter_update_0'))
    parameter_update=log.data.parameter_update_0{:,:};
    flag=parameter_update(:,1);
    
end 
if(isfield(log.data, 'input_rc_0'))
    input_rc=log.data.input_rc_0{:,:};
    [input_rc_N,~]=size(input_rc(:,1));
    input_rc_delta_t=zeros(input_rc_N-1,1);
    for i=1:input_rc_N-1
        input_rc_delta_t(i)=(input_rc(i+1,1))*1e-6-(input_rc(i,1))*1e-6;
    end
    
end 

%% sitl
rate_dowm_simple=20;
att_dowm_simple=10;
att_set_dowm_simple=5;
%% fmu
% rate_dowm_simple=3;
% att_dowm_simple=2;
% att_set_dowm_simple=1;
%%
if(isfield(log.data, 'vehicle_angular_velocity_0'))
    vehicle_angular_velocity=log.data.vehicle_angular_velocity_0{:,:}(1:rate_dowm_simple:end, :);
    [rate_N,~]=size(vehicle_angular_velocity(:,1));
    rate_delta_t=zeros(rate_N-1,1);
    for i=1:rate_N-1
        rate_delta_t(i)=(vehicle_angular_velocity(i+1,1))*1e-6-(vehicle_angular_velocity(i,1))*1e-6;
    end
end 
if(isfield(log.data, 'vehicle_angular_acceleration_0'))
    vehicle_angular_acceleration=log.data.vehicle_angular_acceleration_0{:,:}(1:rate_dowm_simple:end, :);
    [rate_acc_N,~]=size(vehicle_angular_acceleration(:,1));
    rate_acc_delta_t=zeros(rate_acc_N-1,1);
    for i=1:rate_acc_N-1
        rate_acc_delta_t(i)=(vehicle_angular_acceleration(i+1,1))*1e-6-(vehicle_angular_acceleration(i,1))*1e-6;
    end
end 
if(isfield(log.data, 'vehicle_rates_setpoint_0'))
    vehicle_rates_setpoint=log.data.vehicle_rates_setpoint_0{:,:}(1:att_dowm_simple:end, :);
    [rate_setpoint_N,~]=size(vehicle_rates_setpoint(:,1));
    rate_setpoint_delta_t=zeros(rate_setpoint_N-1,1);
    for i=1:rate_setpoint_N-1
        rate_setpoint_delta_t(i)=(vehicle_rates_setpoint(i+1,1))*1e-6-(vehicle_rates_setpoint(i,1))*1e-6;
    end
end 
if(isfield(log.data, 'vehicle_attitude_0'))
    vehicle_attitude=log.data.vehicle_attitude_0{:,:}(1:att_dowm_simple:end, :);
    q_0=vehicle_attitude(:,3);
    q_1=vehicle_attitude(:,4);
    q_2=vehicle_attitude(:,5);
    q_3=vehicle_attitude(:,6);
    Roll=quat_to_roll([q_0 q_1 q_2 q_3]);
    Pitch=quat_to_pitch([q_0 q_1 q_2 q_3]);
    Yaw=quat_to_yaw([q_0 q_1 q_2 q_3]);
    [attitude_N,~]=size(vehicle_attitude(:,1));
    attitude_delta_t=zeros(attitude_N-1,1);
    for i=1:attitude_N-1
        attitude_delta_t(i)=(vehicle_attitude(i+1,1))*1e-6-(vehicle_attitude(i,1))*1e-6;
    end
end 
if(isfield(log.data, 'vehicle_attitude_setpoint_0'))
    vehicle_attitude_setpoint=log.data.vehicle_attitude_setpoint_0{:,:}(1:att_set_dowm_simple:end, :);
    q_0_setpoint=vehicle_attitude_setpoint(:,6);
    q_1_setpoint=vehicle_attitude_setpoint(:,7);
    q_2_setpoint=vehicle_attitude_setpoint(:,8);
    q_3_setpoint=vehicle_attitude_setpoint(:,9);
    Roll_setpoint=quat_to_roll([q_0_setpoint q_1_setpoint q_2_setpoint q_3_setpoint]);
    Pitch_setpoint=quat_to_pitch([q_0_setpoint q_1_setpoint q_2_setpoint q_3_setpoint]);
    Yaw_setpoint=quat_to_yaw([q_0_setpoint q_1_setpoint q_2_setpoint q_3_setpoint]);
    [attitude_setpoint_N,~]=size(vehicle_attitude_setpoint(:,1));
    attitude_setpoint_delta_t=zeros(attitude_setpoint_N-1,1);
    for i=1:attitude_setpoint_N-1
        attitude_setpoint_delta_t(i)=(vehicle_attitude_setpoint(i+1,1))*1e-6-(vehicle_attitude_setpoint(i,1))*1e-6;
    end
end 

if(isfield(log.data, 'vehicle_local_position_0'))
    vehicle_local_position=log.data.vehicle_local_position_0{:,:};
    XYZ=vehicle_local_position(:,6:8);
    V_XYZ=vehicle_local_position(:,12:14);
    [pose_N,~]=size(vehicle_local_position(:,1));
    pose_delta_t=zeros(pose_N-1,1);
    for i=1:pose_N-1
        pose_delta_t(i)=(vehicle_local_position(i+1,1))*1e-6-(vehicle_local_position(i,1))*1e-6;
    end
end 

if(isfield(log.data, 'vehicle_local_position_setpoint_0'))
    vehicle_local_position_setpoint=log.data.vehicle_local_position_setpoint_0{:,:};
    XYZ_setpoint=vehicle_local_position_setpoint(:,2:4);
    V_XYZ_setpoint=vehicle_local_position_setpoint(:,7:9);
    [pose_setpoint_N,~]=size(vehicle_local_position_setpoint(:,1));
    pose_setpoint_delta_t=zeros(pose_setpoint_N-1,1);
    for i=1:pose_setpoint_N-1
        pose_setpoint_delta_t(i)=(vehicle_local_position_setpoint(i+1,1))*1e-6-(vehicle_local_position_setpoint(i,1))*1e-6;
    end
end 

if(isfield(log.data, 'actuator_controls_0_0'))
    actuator_controls=log.data.actuator_controls_0_0{:,:}(1:rate_dowm_simple:end, :);   
    Roll_control=actuator_controls(:,3);
    Pitch_control=actuator_controls(:,4);
    Yaw_control=actuator_controls(:,5);
    if(ismember('indi_fb_0_', log.data.actuator_controls_0_0.Properties.VariableNames))
        indi_feedback=actuator_controls(:,11:13);
        error_feedback=actuator_controls(:,14:16);
    end
    [actuator_N,~]=size(actuator_controls(:,1));
    actuator_delta_t=zeros(actuator_N-1,1);
    actuator_delta=zeros(actuator_N-1,1);
    for i=1:actuator_N-1
        actuator_delta_t(i)=(actuator_controls(i+1,1))*1e-6-(actuator_controls(i,1))*1e-6;
    end
    for i=1:actuator_N-1
        actuator_delta(i)=actuator_controls(i+1,3)-actuator_controls(i,3) ;
    end
end
if(isfield(log.data, 'actuator_outputs_0'))
    actuator_outputs=log.data.actuator_outputs_0{:,:}(1:rate_dowm_simple:end, :);   
    cs1=actuator_outputs(:,7);
    cs2=actuator_outputs(:,8);
    cs3=actuator_outputs(:,9);
    cs4=actuator_outputs(:,10);
    [cs_N,~]=size(actuator_outputs(:,1));
    cs_delta_t=zeros(cs_N-1,1);
    cs_delta=zeros(cs_N-1,1);
    for i=1:cs_N-1
        cs_delta_t(i)=(actuator_outputs(i+1,1))*1e-6-(actuator_outputs(i,1))*1e-6;
    end
    for i=1:cs_N-1
        cs_delta(i)=actuator_outputs(i+1,7)-actuator_outputs(i,7);
    end
end


if(isfield(log.data, 'allocation_value_0'))
    allocation_value=log.data.allocation_value_0{:,:}(1:rate_dowm_simple:end, :);
    [allocation_value_N,~]=size(allocation_value(:,1));
    allocation_value_delta_t=zeros(allocation_value_N-1,1);
    allocation_value_delta=zeros(allocation_value_N-1,1);
    for i=1:allocation_value_N-1
        allocation_value_delta_t(i)=(allocation_value(i+1,1))*1e-6-(allocation_value(i,1))*1e-6;
    end
    for i=1:allocation_value_N-1
        allocation_value_delta(i)=allocation_value(i+1,12)-allocation_value(i,12);
    end
end 

if(isfield(log.data, 'vehicle_visual_odometry_0'))
    vehicle_visual_odometry=log.data.vehicle_visual_odometry_0{:,:};
    visual_odometry_X=vehicle_visual_odometry(:,3);
    visual_odometry_Y=vehicle_visual_odometry(:,4);
    visual_odometry_Z=vehicle_visual_odometry(:,5);
    visual_odometry_q0=vehicle_visual_odometry(:,6);
    visual_odometry_q1=vehicle_visual_odometry(:,7);
    visual_odometry_q2=vehicle_visual_odometry(:,8);
    visual_odometry_q3=vehicle_visual_odometry(:,9);
    
end

%% Disturbance
% ==== Parameter definitions ====
bias_x = 0;          % Bias term
amplitude_x = 0.1;   % Torque amplitude M=0.2 but \nu_h is acceleration
frequency_x = 0.2;   % Frequency (Hz)

bias_y = 0;          % Bias term
amplitude_y = 0.1;   % Torque amplitude M=0.2 but \nu_h is acceleration
frequency_y = 0.4;   % Frequency (Hz)
start_time_sec = 26.8; 

I_x=0.01149;
I_y=0.01153;
I_z=0.00487;
I=[I_x 0 0;0 I_y 0;0 0 I_z];
% ==== Time axis definition ====
t = vehicle_angular_velocity(:,1)*1e-6-vehicle_angular_velocity(1,1)*1e-6 + start_time_sec;          

% ==== Signal formulas ====
% v_h =  -d, such that, we plot -d
tau_x = bias_x + amplitude_x * sin(2 * pi * frequency_x * (t - start_time_sec));
tau_y = bias_y + amplitude_y * sin(2 * pi * frequency_y * (t - start_time_sec));
tau=[tau_x;tau_y;0];
len_t=length(t);
trim=zeros(3,len_t);
for i=1:len_t
    omega_B=vehicle_angular_velocity(i,3:5)';
    trim(:,i)=I\(-cross(omega_B,I*omega_B))+I\[tau_x(i);tau_y(i);0];
end
%%
%  
set(groot, ...
    'defaultAxesFontSize', 8, ...
    'defaultAxesFontName', 'Times New Roman', ...
    'defaultAxesLineWidth', 0.5, ...
    'defaultAxesLabelFontSizeMultiplier', 1, ...
    'defaultAxesTitleFontSizeMultiplier', 1);

fig1 = figure(1);

% Color segment settings (use time as the x-axis background color)
region_edges = [flag(2)*1e-6, flag(6)*1e-6; flag(6)*1e-6, flag(9)*1e-6];%; flag(10)*1e-6 120
region_colors = [
    0.975 0.985 1.000;  
    [0.990, 0.985, 0.965] 
];

% define colors
wc1_color = [0.3 0.45 0.8];
wc2_color = [0.75, 0.38, 0.15];
resp_color = [0.70, 0.22, 0.40];

time_start=25;
time_len=100;

%% subplot 1
subplot(3,1,1); hold on;
% color fill
yl = [-180 180];
for i = 1:size(region_edges,1)
    f=fill([region_edges(i,1) region_edges(i,2) region_edges(i,2) region_edges(i,1)], ...
         [yl(1) yl(1) yl(2) yl(2)], region_colors(i,:), 'EdgeColor', 'none');
    set(f,'HandleVisibility', 'off');
end
% setting lines
plot((vehicle_attitude_setpoint(:,1))*1e-6, Roll_setpoint*r2d, ...
    '-', 'LineWidth', 0.7, 'Color', [0.1, 0.1, 0.1]);
% 
plot((vehicle_attitude(:,1))*1e-6, Roll*r2d, ...
    '--', 'LineWidth', 0.8, 'Color', resp_color);
%flag：
plot([flag(2) flag(2)]*1e-6,[-180 180], '-.', 'Color',wc1_color, 'LineWidth', 0.5);hold on;
text(flag(2)*1e-6+0.3, -22,'$f_{c1}=30$','Color',wc1_color,'FontSize',8,'Interpreter', 'latex');

plot([flag(3) flag(3)]*1e-6,[-180 180], '-.', 'Color',wc1_color, 'LineWidth', 0.5);hold on;
text(flag(3)*1e-6+0.3, -22,'$f_{c1}=20$','Color',wc1_color,'FontSize',8,'Interpreter', 'latex');

plot([flag(4) flag(4)]*1e-6,[-180 180], '-.', 'Color',wc1_color, 'LineWidth', 0.5);hold on;
text(flag(4)*1e-6+0.3, -22,'$f_{c1}=10$','Color',wc1_color,'FontSize',8,'Interpreter', 'latex');

plot([flag(5) flag(5)]*1e-6,[-180 180], '-.', 'Color',wc1_color, 'LineWidth', 0.5);hold on;
text(flag(5)*1e-6+0.3, -22,'$f_{c1}=8$','Color',wc1_color,'FontSize',8,'Interpreter', 'latex');

plot([flag(6) flag(6)]*1e-6,[-180 180], '-.', 'Color',wc2_color, 'LineWidth', 0.5);hold on;
text(flag(6)*1e-6+0.3, -22,'$f_{c2}=20$','Color',wc2_color,'FontSize',8,'Interpreter', 'latex');

plot([flag(7) flag(7)]*1e-6,[-180 180], '-.', 'Color',wc2_color, 'LineWidth', 0.5);hold on;
text(flag(7)*1e-6+0.3, -22,'$f_{c2}=8$','Color',wc2_color,'FontSize',8,'Interpreter', 'latex');

plot([flag(8) flag(8)]*1e-6,[-180 180], '-.', 'Color',wc2_color, 'LineWidth', 0.5);hold on;
text(flag(8)*1e-6+0.3, -22,'$f_{c2}=1$','Color',wc2_color,'FontSize',8,'Interpreter', 'latex');

plot([flag(9) flag(9)]*1e-6,[-180 180], '-.', 'Color',wc2_color, 'LineWidth', 0.5);hold on;
text(flag(9)*1e-6+0.3, -22,'$u_0=0$','Color',wc2_color,'FontSize',8,'Interpreter', 'latex');



grid on;set(gca, 'Layer', 'top')  % Force grid to display on top layer
axis([time_start time_len -30 30]);
xticks(time_start:5:time_len);
yticks(-30:10:30);
xlabel({'Time (s)'});
ylabel({'Roll angle $\varphi$ (deg)'},'Interpreter', 'latex');
% title('Euler angular', 'FontSize', 8,'FontName','Times New Roman');
legend('Setpoint', 'Response','NumColumns', 2, 'Location', 'northeast', 'FontSize', 8,'FontName','Times New Roman');

%% subplot 2
subplot(3,1,2); hold on;
% Background color filling
yl = [-180 180];
for i = 1:size(region_edges,1)
    f=fill([region_edges(i,1) region_edges(i,2) region_edges(i,2) region_edges(i,1)], ...
         [yl(1) yl(1) yl(2) yl(2)], region_colors(i,:), 'EdgeColor', 'none');
    set(f,'HandleVisibility', 'off');
end

plot((vehicle_rates_setpoint(:,1))*1e-6, vehicle_rates_setpoint(:,2)*r2d, '-', 'LineWidth', 0.7, 'Color', [0.1, 0.1, 0.1]);
plot((vehicle_angular_velocity(:,1))*1e-6, vehicle_angular_velocity(:,3)*r2d, '--', 'LineWidth', 0.7, 'Color', resp_color);

%标识：
plot([flag(2) flag(2)]*1e-6,[-180 180], '-.', 'Color',wc1_color, 'LineWidth', 0.5);hold on;
text(flag(2)*1e-6+0.3, -62,'$f_{c1}=30$','Color',wc1_color,'FontSize',8,'Interpreter', 'latex');

plot([flag(3) flag(3)]*1e-6,[-180 180], '-.', 'Color',wc1_color, 'LineWidth', 0.5);hold on;
text(flag(3)*1e-6+0.3, -62,'$f_{c1}=20$','Color',wc1_color,'FontSize',8,'Interpreter', 'latex');

plot([flag(4) flag(4)]*1e-6,[-180 180], '-.', 'Color',wc1_color, 'LineWidth', 0.5);hold on;
text(flag(4)*1e-6+0.3, -62,'$f_{c1}=10$','Color',wc1_color,'FontSize',8,'Interpreter', 'latex');

plot([flag(5) flag(5)]*1e-6,[-180 180], '-.', 'Color',wc1_color, 'LineWidth', 0.5);hold on;
text(flag(5)*1e-6+0.3, -62,'$f_{c1}=8$','Color',wc1_color,'FontSize',8,'Interpreter', 'latex');

plot([flag(6) flag(6)]*1e-6,[-180 180], '-.', 'Color',wc2_color, 'LineWidth', 0.5);hold on;
text(flag(6)*1e-6+0.3, -62,'$f_{c2}=20$','Color',wc2_color,'FontSize',8,'Interpreter', 'latex');

plot([flag(7) flag(7)]*1e-6,[-180 180], '-.', 'Color',wc2_color, 'LineWidth', 0.5);hold on;
text(flag(7)*1e-6+0.3, -62,'$f_{c2}=8$','Color',wc2_color,'FontSize',8,'Interpreter', 'latex');

plot([flag(8) flag(8)]*1e-6,[-180 180], '-.', 'Color',wc2_color, 'LineWidth', 0.5);hold on;
text(flag(8)*1e-6+0.3, -62,'$f_{c2}=1$','Color',wc2_color,'FontSize',8,'Interpreter', 'latex');

plot([flag(9) flag(9)]*1e-6,[-180 180], '-.', 'Color',wc2_color, 'LineWidth', 0.5);hold on;
text(flag(9)*1e-6+0.3, -62,'$u_0=0$','Color',wc2_color,'FontSize',8,'Interpreter', 'latex');

grid on;set(gca, 'Layer', 'top')   
axis([time_start time_len -80 80]);
xticks(time_start:5:time_len);
yticks(-80:20:80);
xlabel({'Time (s)'});
ylabel({'Roll rate $p$ (deg/s)'},'Interpreter', 'latex');
% title('Angular rate', 'FontSize', 8,'FontName','Times New Roman');
legend('Setpoint', 'Response', 'NumColumns', 2, 'Location', 'northeast', 'FontSize', 8,'FontName','Times New Roman');

%% subplot 3
subplot(3,1,3); hold on;
% Background color filling
yl = [-180 180];
for i = 1:size(region_edges,1)
    f=fill([region_edges(i,1) region_edges(i,2) region_edges(i,2) region_edges(i,1)], ...
         [yl(1) yl(1) yl(2) yl(2)], region_colors(i,:), 'EdgeColor', 'none');
    set(f,'HandleVisibility', 'off');
end

plot((vehicle_angular_acceleration(:,1))*1e-6, vehicle_angular_acceleration(:,3), '--', 'LineWidth', 0.7, 'Color', resp_color);
%标识：
plot([flag(2) flag(2)]*1e-6,[-180 180], '-.', 'Color',wc1_color, 'LineWidth', 0.5');hold on;
text(flag(2)*1e-6+0.3, -15,'$f_{c1}=30$','Color',wc1_color,'FontSize',8,'Interpreter', 'latex');

plot([flag(3) flag(3)]*1e-6,[-180 180], '-.', 'Color',wc1_color, 'LineWidth', 0.5');hold on;
text(flag(3)*1e-6+0.3, -15,'$f_{c1}=20$','Color',wc1_color,'FontSize',8,'Interpreter', 'latex');

plot([flag(4) flag(4)]*1e-6,[-180 180], '-.', 'Color',wc1_color, 'LineWidth', 0.5');hold on;
text(flag(4)*1e-6+0.3, -15,'$f_{c1}=10$','Color',wc1_color,'FontSize',8,'Interpreter', 'latex');

plot([flag(5) flag(5)]*1e-6,[-180 180], '-.', 'Color',wc1_color, 'LineWidth', 0.5');hold on;
text(flag(5)*1e-6+0.3, -15,'$f_{c1}=8$','Color',wc1_color,'FontSize',8,'Interpreter', 'latex');

plot([flag(6) flag(6)]*1e-6,[-180 180], '-.', 'Color',wc2_color, 'LineWidth', 0.5');hold on;
text(flag(6)*1e-6+0.3, -15,'$f_{c2}=20$','Color',wc2_color,'FontSize',8,'Interpreter', 'latex');

plot([flag(7) flag(7)]*1e-6,[-180 180], '-.', 'Color',wc2_color, 'LineWidth', 0.5');hold on;
text(flag(7)*1e-6+0.3, -15,'$f_{c2}=8$','Color',wc2_color,'FontSize',8,'Interpreter', 'latex');

plot([flag(8) flag(8)]*1e-6,[-180 180], '-.', 'Color',wc2_color, 'LineWidth', 0.5');hold on;
text(flag(8)*1e-6+0.3, -15,'$f_{c2}=1$','Color',wc2_color,'FontSize',8,'Interpreter', 'latex');

plot([flag(9) flag(9)]*1e-6,[-180 180], '-.', 'Color',wc2_color, 'LineWidth', 0.5');hold on;
text(flag(9)*1e-6+0.3, -15,'$u_0=0$','Color',wc2_color,'FontSize',8,'Interpreter', 'latex');

grid on;set(gca, 'Layer', 'top')   
axis([time_start time_len -20 20]);
xticks(time_start:5:time_len);
yticks(-20:10:20);
% title('Angular Acceleration', 'FontSize', 8,'FontName','Times New Roman');
xlabel({'Time (s)'});
ylabel({'$\dot{p}$ (deg/s$^2$)'},'Interpreter', 'latex');
legend('$\dot{p}$', 'Interpreter', 'latex', 'Location', 'northeast', 'FontSize', 8);

% sgtitle('Angle velocity tracking under different setups', 'FontSize', 8,'FontName','Times New Roman');
PlotToFileColorPDF(fig1, 'results/Figure_9.pdf', 13.5, 11.5); %states_with_noise
% PlotToFileColorPNG(fig1, 'results/states_with_noise.png', 25, 20);

 %% 
fig2 = figure(2);
control_color=[0.72, 0.48, 0.88];
% ==== subplot1: indi_feedback ====
subplot(3,1,1);hold on;
% Background color filling
yl = [-40 40];
for i = 1:size(region_edges,1)
    f=fill([region_edges(i,1) region_edges(i,2) region_edges(i,2) region_edges(i,1)], ...
         [yl(1) yl(1) yl(2) yl(2)], region_colors(i,:), 'EdgeColor', 'none');
    set(f,'HandleVisibility', 'off');
end
plot((actuator_controls(:,1))*1e-6, -indi_feedback(:,1),'--','Color',control_color, 'LineWidth', 0.7); hold on;
plot(t, trim(1,:), '-', 'LineWidth', 0.7, 'Color', [0.1, 0.1, 0.1]);hold on;% , 
%标识：
plot([flag(2) flag(2)]*1e-6,[-180 180], '-.', 'Color',wc1_color);
text(flag(2)*1e-6+0.3, -30,'$f_{c1}=30$','Color',wc1_color,'FontSize',8,'Interpreter', 'latex');

plot([flag(3) flag(3)]*1e-6,[-180 180], '-.', 'Color',wc1_color, 'LineWidth', 0.5');hold on;
text(flag(3)*1e-6+0.3, -30,'$f_{c1}=20$','Color',wc1_color,'FontSize',8,'Interpreter', 'latex');

plot([flag(4) flag(4)]*1e-6,[-180 180], '-.', 'Color',wc1_color, 'LineWidth', 0.5');hold on;
text(flag(4)*1e-6+0.3, -30,'$f_{c1}=10$','Color',wc1_color,'FontSize',8,'Interpreter', 'latex');

plot([flag(5) flag(5)]*1e-6,[-180 180], '-.', 'Color',wc1_color, 'LineWidth', 0.5');hold on;
text(flag(5)*1e-6+0.3, -30,'$f_{c1}=8$','Color',wc1_color,'FontSize',8,'Interpreter', 'latex');

plot([flag(6) flag(6)]*1e-6,[-180 180], '-.', 'Color',wc2_color, 'LineWidth', 0.5');hold on;
text(flag(6)*1e-6+0.3, -30,'$f_{c2}=20$','Color',wc2_color,'FontSize',8,'Interpreter', 'latex');

plot([flag(7) flag(7)]*1e-6,[-180 180], '-.', 'Color',wc2_color, 'LineWidth', 0.5');hold on;
text(flag(7)*1e-6+0.3, -30,'$f_{c2}=8$','Color',wc2_color,'FontSize',8,'Interpreter', 'latex');

plot([flag(8) flag(8)]*1e-6,[-180 180], '-.', 'Color',wc2_color, 'LineWidth', 0.5');hold on;
text(flag(8)*1e-6+0.3, -30,'$f_{c2}=1$','Color',wc2_color,'FontSize',8,'Interpreter', 'latex');

plot([flag(9) flag(9)]*1e-6,[-180 180], '-.', 'Color',wc2_color, 'LineWidth', 0.5');hold on;
text(flag(9)*1e-6+0.3, -30,'$u_0=0$','Color',wc2_color,'FontSize',8,'Interpreter', 'latex');


grid on;set(gca, 'Layer', 'top')   
axis([time_start time_len -40 40]);
xticks(time_start:5:time_len);
yticks(-40:20:40);
legend('$(-\hat{\nu}_h)$','$\alpha+\mathcal{D}d$', 'Interpreter', 'latex','NumColumns', 2, 'Location', 'northwest', 'FontSize', 8);
% title('Trim term estimate', 'FontSize', 8,'FontName','Times New Roman');
xlabel({'Time (s)'});
ylabel('Roll channel');

% ==== subplot2: error_feedback ====
subplot(3,1,2);hold on;
% Background color filling
yl = [-40 40];
for i = 1:size(region_edges,1)
    f=fill([region_edges(i,1) region_edges(i,2) region_edges(i,2) region_edges(i,1)], ...
         [yl(1) yl(1) yl(2) yl(2)], region_colors(i,:), 'EdgeColor', 'none');
    set(f,'HandleVisibility', 'off');
end
plot((actuator_controls(:,1))*1e-6, error_feedback(:,1), '--','Color',control_color, 'LineWidth', 0.7); hold on;
%标识：
plot([flag(2) flag(2)]*1e-6,[-180 180], '-.', 'Color',wc1_color, 'LineWidth', 0.5');hold on;
text(flag(2)*1e-6+0.3, -15,'$f_{c1}=30$','Color',wc1_color,'FontSize',8,'Interpreter', 'latex');

plot([flag(3) flag(3)]*1e-6,[-180 180], '-.', 'Color',wc1_color, 'LineWidth', 0.5');hold on;
text(flag(3)*1e-6+0.3, -15,'$f_{c1}=20$','Color',wc1_color,'FontSize',8,'Interpreter', 'latex');

plot([flag(4) flag(4)]*1e-6,[-180 180], '-.', 'Color',wc1_color, 'LineWidth', 0.5');hold on;
text(flag(4)*1e-6+0.3, -15,'$f_{c1}=10$','Color',wc1_color,'FontSize',8,'Interpreter', 'latex');

plot([flag(5) flag(5)]*1e-6,[-180 180], '-.', 'Color',wc1_color, 'LineWidth', 0.5');hold on;
text(flag(5)*1e-6+0.3, -15,'$f_{c1}=8$','Color',wc1_color,'FontSize',8,'Interpreter', 'latex');

plot([flag(6) flag(6)]*1e-6,[-180 180], '-.', 'Color',wc2_color, 'LineWidth', 0.5');hold on;
text(flag(6)*1e-6+0.3, -15,'$f_{c2}=20$','Color',wc2_color,'FontSize',8,'Interpreter', 'latex');

plot([flag(7) flag(7)]*1e-6,[-180 180], '-.', 'Color',wc2_color, 'LineWidth', 0.5');hold on;
text(flag(7)*1e-6+0.3, -15,'$f_{c2}=8$','Color',wc2_color,'FontSize',8,'Interpreter', 'latex');

plot([flag(8) flag(8)]*1e-6,[-180 180], '-.', 'Color',wc2_color, 'LineWidth', 0.5');hold on;
text(flag(8)*1e-6+0.3, -15,'$f_{c2}=1$','Color',wc2_color,'FontSize',8,'Interpreter', 'latex');

plot([flag(9) flag(9)]*1e-6,[-180 180], '-.', 'Color',wc2_color, 'LineWidth', 0.5');hold on;
text(flag(9)*1e-6+0.3, -15,'$u_0=0$','Color',wc2_color,'FontSize',8,'Interpreter', 'latex');


grid on;set(gca, 'Layer', 'top')   
axis([time_start time_len -20 20]);
xticks(time_start:5:time_len);
yticks(-20:10:20);
% title('Error feedback control', 'FontSize', 8,'FontName','Times New Roman');
legend('$\hat{\nu}_c$', 'Interpreter', 'latex','NumColumns', 1, 'Location', 'northwest', 'FontSize', 8);
xlabel({'Time (s)'});
ylabel('Roll channel');

% ==== subplot3: cs1 ====in simulation = outputs (rad) = _u_cmd = _u_estimate[i] = allocation_value.u_ultimate; _u_estimate is the real value of control input.
subplot(3,1,3);hold on;
% Background color filling
yl = [-500 2500];
for i = 1:size(region_edges,1)
    f=fill([region_edges(i,1) region_edges(i,2) region_edges(i,2) region_edges(i,1)], ...
         [yl(1) yl(1) yl(2) yl(2)], region_colors(i,:), 'EdgeColor', 'none');
    set(f,'HandleVisibility', 'off');
end
% plot((actuator_outputs(:,1))*1e-6, cs1(:,1), '--','Color',control_color, 'LineWidth', 0.7); hold on; 
plot((allocation_value(:,1))*1e-6, allocation_value(:,16)*r2d, '--','Color',control_color, 'LineWidth', 0.7); hold on;% allocation_value \in [-0.3491 0.3491] 
% pwm_max=2000;
% pwm_min=1000;
% plot((allocation_value(:,1))*1e-6, (allocation_value(:,16)/0.3491)*(pwm_max-pwm_min)/2+(pwm_max+pwm_min)/2, '-', 'LineWidth', 0.7, 'Color', [0.1, 0.1, 0.1]); %=allocation_value  In /src/lib/output_limit/output_limit.cpp#L208
 %标识：
plot([flag(2) flag(2)]*1e-6,[-30 30],'-.', 'Color',wc1_color, 'LineWidth', 0.5');hold on;
text(flag(2)*1e-6+0.3, -25,'$f_{c1}=30$','Color',wc1_color,'FontSize',8,'Interpreter', 'latex');

plot([flag(3) flag(3)]*1e-6,[-30 30],'-.', 'Color',wc1_color, 'LineWidth', 0.5');hold on;
text(flag(3)*1e-6+0.3, -25,'$f_{c1}=20$','Color',wc1_color,'FontSize',8,'Interpreter', 'latex');

plot([flag(4) flag(4)]*1e-6,[-30 30],'-.', 'Color',wc1_color, 'LineWidth', 0.5');hold on;
text(flag(4)*1e-6+0.3, -25,'$f_{c1}=10$','Color',wc1_color,'FontSize',8,'Interpreter', 'latex');

plot([flag(5) flag(5)]*1e-6,[-30 30],'-.', 'Color',wc1_color, 'LineWidth', 0.5');hold on;
text(flag(5)*1e-6+0.3, -25,'$f_{c1}=8$','Color',wc1_color,'FontSize',8,'Interpreter', 'latex');

plot([flag(6) flag(6)]*1e-6,[-30 30],'-.', 'Color',wc2_color, 'LineWidth', 0.5');hold on;
text(flag(6)*1e-6+0.3, -25,'$f_{c2}=20$','Color',wc2_color,'FontSize',8,'Interpreter', 'latex');

plot([flag(7) flag(7)]*1e-6,[-30 30],'-.', 'Color',wc2_color, 'LineWidth', 0.5');hold on;
text(flag(7)*1e-6+0.3, -25,'$f_{c2}=8$','Color',wc2_color,'FontSize',8,'Interpreter', 'latex');

plot([flag(8) flag(8)]*1e-6,[-30 30],'-.', 'Color',wc2_color, 'LineWidth', 0.5');hold on;
text(flag(8)*1e-6+0.3, -25,'$f_{c2}=1$','Color',wc2_color,'FontSize',8,'Interpreter', 'latex');

plot([flag(9) flag(9)]*1e-6,[-30 30],'-.', 'Color',wc2_color, 'LineWidth', 0.5');hold on;
text(flag(9)*1e-6+0.3, -25,'$u_0=0$','Color',wc2_color,'FontSize',8,'Interpreter', 'latex');



grid on;set(gca, 'Layer', 'top')   
% axis([time_start time_len 800 2000]);
axis([time_start time_len -30 20]);
xticks(time_start:5:time_len);
% yticks(800:200:2000);
yticks(-30:10:20);
legend('$u_1$', 'Interpreter', 'latex','NumColumns', 1, 'Location', 'northwest', 'FontSize', 8);
% title('Control input', 'FontSize', 8,'FontName','Times New Roman');
xlabel({'Time (s)'});
ylabel('Control input (deg)');
% sgtitle('Control input under different setups', 'FontSize', 8,'FontName','Times New Roman');

PlotToFileColorPDF(fig2, 'results/Figure_10.pdf', 13.5, 11.5); % control_with_noise
% PlotToFileColorPNG(fig2, 'results/control_with_noise.png', 25, 20);





fig3 = figure(3);
hold on;


% Timestamps
t_us = vehicle_local_position(:,1);

% Segmented event times
event = [flag(2), flag(6), flag(9)];

% Get segment indices
idx1 = find(t_us >= time_start*1e6 & t_us <= event(1));
idx2 = find(t_us >= event(1) & t_us <= event(2));
idx3 = find(t_us >= event(2) & t_us <= event(3));
idx4 = find(t_us >= event(3) & t_us <= time_len*1e6);
idx  = find(t_us >= time_start*1e6 & t_us < time_len*1e6);

% Original reference trajectory (non-segmented) — XY only
plot(XYZ_setpoint(idx,1), XYZ_setpoint(idx,2), ...
    's', ...                            % Square marker for better visibility
    'MarkerSize', 8, ...                % Increase marker size
    'MarkerEdgeColor', [0 0 0], ...     % Black border
    'MarkerFaceColor', [1 0.85 0], ...  % Golden fill, more eye-catching
    'LineStyle', 'none');               % No connecting line

% Color settings
head_color = [0.35, 0.70, 0.75];
tail_color = [0.45, 0.30, 0.45];

% Segmented trajectory plotting (XY only)
if ~isempty(idx1)
    plot(XYZ(idx1,1), XYZ(idx1,2), '-', ...
        'Color', head_color, 'LineWidth', 0.5);
end
if ~isempty(idx2)
    plot(XYZ(idx2,1), XYZ(idx2,2), '--', ...
        'Color', wc1_color, 'LineWidth', 1);
end
if ~isempty(idx3)
    plot(XYZ(idx3,1), XYZ(idx3,2), '-', ...
        'Color', wc2_color, 'LineWidth', 0.75);
end
if ~isempty(idx4)
    plot(XYZ(idx4,1), XYZ(idx4,2), '-.', ...
        'Color', tail_color, 'LineWidth', 0.5);
end

% Figure settings (no Z-axis)
% title('Trajectory in XY plane', 'FontSize', 8,'FontName','Times New Roman');
xlabel('X (m)');
ylabel('Y (m)');

legend('Setpoint', 'Stage 1', 'Stage 2', ...
       'Stage 3', 'Stage 4', ...
       'Interpreter', 'latex', ...
       'Location', 'southeast', ...
       'NumColumns', 2,...
       'FontSize', 8);

grid on;
axis([-1.5 1.5 -0.8 0.8]);  % 仅 XY
xticks(-1.5:0.5:1.5);
yticks(-0.8:0.4:0.8);

PlotToFileColorPDF(fig3, 'results/Figure_11.pdf', 7.5, 7.5); % noise_test_position
% PlotToFileColorPNG(fig3, 'results/noise_test_position.png', 20, 15);

%% 
if(isfield(log.data, 'vehicle_angular_velocity_0'))
    % figure,
    % plot(1:rate_N-1, rate_delta_t,'k-','LineWidth',1);hold on;
    % ylabel('time (s)');
    % title('vehicle angular velocity');
    disp('mean(rate_delta_t)');
    mean(rate_delta_t)/rate_dowm_simple
end
if(isfield(log.data, 'vehicle_rates_setpoint_0'))
    % figure,
    % plot(1:rate_setpoint_N-1, rate_setpoint_delta_t,'k-','LineWidth',1);hold on;
    % ylabel('time (s)');
    % title('vehicle rate setpoint');
    disp('mean(rate_setpoint_delta_t)');
    mean(rate_setpoint_delta_t)/att_dowm_simple
end
% if(isfield(log.data, 'vehicle_angular_acceleration_0'))
%     % figure,
%     % plot(1:rate_acc_N-1, rate_acc_delta_t,'k-','LineWidth',1);hold on;
%     ylabel('time (s)');
%     title('vehicle angular acceleration');
%     disp('mean(rate_acc_delta_t)');
%     mean(rate_acc_delta_t)
% end
if(isfield(log.data, 'vehicle_attitude_0'))
    % figure,
    % plot(1:attitude_N-1, attitude_delta_t,'k-','LineWidth',1);hold on;
    % ylabel('time (s)');
    % title('vehicle attitude');
    disp('mean(attitude_delta_t)');
    mean(attitude_delta_t)/att_dowm_simple

end
if(isfield(log.data, 'vehicle_attitude_setpoint_0'))
    % figure,
    % plot(1:attitude_setpoint_N-1, attitude_setpoint_delta_t,'k-','LineWidth',1);hold on;
    % ylabel('time (s)');
    % title('vehicle attitude setpoint');
    disp('mean(attitude_setpoint_delta_t)');
    mean(attitude_setpoint_delta_t)/att_set_dowm_simple

end
if(isfield(log.data, 'vehicle_local_position_0'))
    % figure,
    % plot(1:pose_N-1, pose_delta_t,'k-','LineWidth',1);hold on;
    % ylabel('time (s)');
    % title('vehicle local position');
    disp('mean(pose_delta_t)');
    mean(pose_delta_t)
end
if(isfield(log.data, 'vehicle_local_position_setpoint_0'))
    % figure,
    % plot(1:pose_setpoint_N-1, pose_setpoint_delta_t,'k-','LineWidth',1);hold on;
    % ylabel('time (s)');
    % title('vehicle local position setpoint');
    disp('mean(pose_setpoint_delta_t)');
    mean(pose_setpoint_delta_t)
end
if(isfield(log.data, 'actuator_controls_0_0'))
    % figure,
    % plot(1:actuator_N-1, actuator_delta_t,'k-','LineWidth',1);hold on;
    % ylabel('time (s)');
    % title('actuator controls');
    disp('mean(actuator_delta_t)');
    mean(actuator_delta_t)/rate_dowm_simple
end 
if(isfield(log.data, 'actuator_outputs_0'))
    % figure,
    % plot(1:cs_N-1, cs_delta_t,'k-','LineWidth',1);hold on;
    % ylabel('time (s)');
    % title('actuator outputs');
    disp('mean(cs_delta_t)');
    mean(cs_delta_t)/rate_dowm_simple
end

