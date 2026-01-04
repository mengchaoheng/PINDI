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
ulgFileName = 'data/log_40_2025-6-24-14-58-42';
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
% rate_dowm_simple=25;
% att_dowm_simple=10;
% att_set_dowm_simple=5;
%% fmu
rate_dowm_simple=3;
att_dowm_simple=2;
att_set_dowm_simple=1;
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
% /**
%  * USER_AC_METHOD
%  *
%  * use priority control allocation or not.
%  *
%  * @value 0 inv
%  * @value 1 WLS
%  * @value 2 DIR
%  * @value 3 PCA
%  * @group Mixer Output
%  */
% PARAM_DEFINE_INT32(USER_AC_METHOD, 0);
% /**
% * USER_DIST_MAG
% *
% * The magnitude of the disturbance added to the control surfaces.
% *
% * @min 0
% * @max 0.3491
% * @unit rad
% * @decimal 4
% * @increment 0.0001
% * @group Mixer Output
% */
% PARAM_DEFINE_FLOAT(USER_DIST_MAG, 0.0f);
%=========
% flag(1)=USER_DIST_MAG=0.07  by default USER_AC_METHOD=0 
% flag(2)=USER_DIST_MAG=0.09  by default USER_AC_METHOD=0 
% flag(3)=USER_DIST_MAG=0.08  by default USER_AC_METHOD=0
% flag(4)=USER_AC_METHOD=1
% flag(5)=USER_AC_METHOD=2
% flag(6)=USER_AC_METHOD=3 (second time)
% // rc detection
% // Upper value is -1
% // channels[6]:  -1   0   1   = yaw step  // Channel 7 top-right switch
% // Channels 9–12 are on the front side
% // channels[8]:  -1   0   1   = servo disturbance
% // channels[9]:  -1   0   1   =
% // channels[10]: -1   0   1   =
% // channels[11]: -1   0   1   = PID or INDI mode
% channels[i] corresponds to values_i_ in MATLAB log.data.input_rc_0
% Therefore, the event corresponds to changes in channels[6], 
% specifically in the segment after flag(3)=USER_DIST_MAG=0.08.
% The step input trigger points are identified manually:
% vehicle_attitude_setpoint(15703,1)  vehicle_attitude_setpoint(17487,1)  vehicle_attitude_setpoint(19257,1)  vehicle_attitude_setpoint(22469,1)
%% Parameter initialization


% event = [flag(3), flag(4), flag(5), second test of flag(6)];

step_time = 2e6;  % μs
event = [vehicle_attitude_setpoint(15703,1)  vehicle_attitude_setpoint(17487,1)  vehicle_attitude_setpoint(19257,1) vehicle_attitude_setpoint(22469,1)]-step_time;
Time = 5 * step_time;

t_set = vehicle_rates_setpoint(:,1);              % μs
pqr_set = rad2deg(vehicle_rates_setpoint(:,2:4)); % deg/s

t_meas = vehicle_angular_velocity(:,1);           % μs
pqr_meas = rad2deg(vehicle_angular_velocity(:,3:5));

n_event = length(event);
err = cell(n_event,1);
t_rel = cell(n_event,1);

%% Interpolation + Error Calculation
for k = 1:n_event
    t0 = event(k) + 0.5 * step_time;
    t1 = t0 + Time - 1  * step_time;

    % Current data window
    idx_set  = (t_set  >= t0) & (t_set  <= t1);
    idx_meas = (t_meas >= t0) & (t_meas <= t1);

    ts_set  = (t_set(idx_set) - t0) * 1e-6;   % s
    ts_meas = (t_meas(idx_meas) - t0) * 1e-6; % s

    d_set  = pqr_set(idx_set,:);
    d_meas = pqr_meas(idx_meas,:);

    % Unified interpolation method
    [err{k},t_rel{k}] = interpolate_and_error(ts_set, d_set, ts_meas, d_meas);
end

% Colors
ref_color  = [0.93, 0.70, 0.15];   % Ref - golden brown
inv_color  = [0.25, 0.47, 0.85];   % INV - deep blue
dir_color  = [0.60, 0.32, 0.75];   % DIR - purple-gray
pca_color  = [0.80, 0.40, 0.10];   % PCA - dark orange
wls_color  = [0.25, 0.25, 0.25];   % WLS - dark gray
%%  
set(groot, ...
    'defaultAxesFontSize', 8, ...
    'defaultAxesFontName', 'Times New Roman', ...
    'defaultAxesLineWidth', 0.5, ...
    'defaultAxesLabelFontSizeMultiplier', 1, ...
    'defaultAxesTitleFontSizeMultiplier', 1);
%%
fig1 = figure(1);

subplot(3,1,1); hold on; grid on;
plot(t_rel{1}, err{1}(:,1), '--', 'Color', inv_color, 'LineWidth', 0.5);
plot(t_rel{2}, err{2}(:,1), ':',  'Color', wls_color, 'LineWidth', 0.5);
plot(t_rel{3}, err{3}(:,1), '-.', 'Color', dir_color, 'LineWidth', 0.5);
plot(t_rel{4}, err{4}(:,1), '-',  'Color', pca_color, 'LineWidth', 0.5);

% title('Roll channel', 'FontSize', 8,'FontName','Times New Roman');
xlabel({'Time (s)'});
ylabel('p error (deg/s)');
axis([0 8 -200 200]); xticks(0:1:8);yticks(-200:100:200);
legend('INDI+INV','INDI+WLS','INDI+DIR','PINDI', 'NumColumns', 2, 'Location', 'southeast','FontSize', 6,'FontName','Times New Roman');

subplot(3,1,2); hold on; grid on;
plot(t_rel{1}, err{1}(:,2), '--', 'Color', inv_color, 'LineWidth', 0.5);
plot(t_rel{2}, err{2}(:,2), ':',  'Color', wls_color, 'LineWidth', 0.5);
plot(t_rel{3}, err{3}(:,2), '-.', 'Color', dir_color, 'LineWidth', 0.5);
plot(t_rel{4}, err{4}(:,2), '-',  'Color', pca_color, 'LineWidth', 0.5);

% title('Pitch channel', 'FontSize', 8,'FontName','Times New Roman');
xlabel({'Time (s)'});
ylabel('q error (deg/s)');
axis([0 8 -90 90]); xticks(0:1:8);yticks(-90:30:90);

subplot(3,1,3); hold on; grid on;
plot(t_rel{1}, err{1}(:,3), '--', 'Color', inv_color, 'LineWidth', 0.5);
plot(t_rel{2}, err{2}(:,3), ':',  'Color', wls_color, 'LineWidth', 0.5);
plot(t_rel{3}, err{3}(:,3), '-.', 'Color', dir_color, 'LineWidth', 0.5);
plot(t_rel{4}, err{4}(:,3), '-',  'Color', pca_color, 'LineWidth', 0.5);

% title('Yaw channel', 'FontSize', 8,'FontName','Times New Roman');
xlabel({'Time (s)'});
ylabel('r error (deg/s)');
axis([0 8 -360 360]); xticks(0:1:8);yticks(-360:120:360);



% sgtitle('Angle velocity tracking error under different control', 'FontSize', 8,'FontName','Times New Roman');

PlotToFileColorPDF(fig1,'results/Figure_20a.pdf',6.5,10.5); % flight_rate_tracking

%% Attitude
% Time axis (μs)
t_set_att   = vehicle_attitude_setpoint(:,1);
t_meas_att  = vehicle_attitude(:,1);

% Attitude angles (deg)
att_set  = rad2deg([Roll_setpoint, Pitch_setpoint, Yaw_setpoint]);
att_meas = rad2deg([Roll, Pitch, Yaw]);

% Initialize attitude error results
err_att = cell(n_event,1);
t_rel_att = cell(n_event,1);

% === Interpolation + Error Calculation: Attitude ===
for k = 1:n_event
    t0 = event(k) + 0.5 * step_time;
    t1 = t0 + Time - 1  * step_time;

    % Current data window (attitude)
    idx_set_att  = (t_set_att  >= t0) & (t_set_att  <= t1);
    idx_meas_att = (t_meas_att >= t0) & (t_meas_att <= t1);

    ts_set_att  = (t_set_att(idx_set_att) - t0) * 1e-6;   % s
    ts_meas_att = (t_meas_att(idx_meas_att) - t0) * 1e-6; % s

    d_set_att  = att_set(idx_set_att,:);
    d_meas_att = att_meas(idx_meas_att,:);

    % Interpolation & Error
    [err_att{k}, t_rel_att{k}] = interpolate_and_error(ts_set_att, d_set_att, ts_meas_att, d_meas_att);
end

%%
fig2 = figure(2);

% --- Roll (angle error) ---
subplot(3,1,1); hold on; grid on;
plot(t_rel_att{1}, err_att{1}(:,1), '--', 'Color', inv_color, 'LineWidth', 0.5);
plot(t_rel_att{2}, err_att{2}(:,1), ':',  'Color', wls_color, 'LineWidth', 0.5);
plot(t_rel_att{3}, err_att{3}(:,1), '-.', 'Color', dir_color, 'LineWidth', 0.5);
plot(t_rel_att{4}, err_att{4}(:,1), '-',  'Color', pca_color, 'LineWidth', 0.5);

% title('Roll angle tracking error', 'FontSize', 8,'FontName','Times New Roman');
ylabel('Roll error (deg)');
xlabel('Time (s)');
axis([0 8 -40 40]); xticks(0:1:8);yticks(-40:20:40);
legend('INDI+INV','INDI+WLS','INDI+DIR','PINDI', ...
    'NumColumns', 2, 'Location', 'southeast', 'FontSize', 6, 'FontName', 'Times New Roman');

% --- Pitch ---
subplot(3,1,2); hold on; grid on;
plot(t_rel_att{1}, err_att{1}(:,2), '--', 'Color', inv_color, 'LineWidth', 0.5);
plot(t_rel_att{2}, err_att{2}(:,2), ':',  'Color', wls_color, 'LineWidth', 0.5);
plot(t_rel_att{3}, err_att{3}(:,2), '-.', 'Color', dir_color, 'LineWidth', 0.5);
plot(t_rel_att{4}, err_att{4}(:,2), '-',  'Color', pca_color, 'LineWidth', 0.5);

% title('Pitch angle tracking error', 'FontSize', 8,'FontName','Times New Roman');
ylabel('Pitch error (deg)');
xlabel('Time (s)');
axis([0 8 -20 20]); xticks(0:1:8);yticks(-180:10:180);

% --- Yaw ---
subplot(3,1,3); hold on; grid on;
plot(t_rel_att{1}, err_att{1}(:,3), '--', 'Color', inv_color, 'LineWidth', 0.5);
plot(t_rel_att{2}, err_att{2}(:,3), ':',  'Color', wls_color, 'LineWidth', 0.5);
plot(t_rel_att{3}, err_att{3}(:,3), '-.', 'Color', dir_color, 'LineWidth', 0.5);
plot(t_rel_att{4}, err_att{4}(:,3), '-',  'Color', pca_color, 'LineWidth', 0.5);

% title('Yaw angle tracking error', 'FontSize', 8,'FontName','Times New Roman');
ylabel('Yaw error (deg)');
xlabel('Time (s)');
axis([0 8 -50 100]); xticks(0:1:8);yticks(-50:50:100);

% sgtitle('Euler angle tracking error under different control strategies', ...
        % 'FontSize', 8, 'FontName', 'Times New Roman');

PlotToFileColorPDF(fig2,'results/Figure_20b.pdf',6.5,10.5); % flight_att_tracking


t_alloc = allocation_value(:,1);            % μs
alloc_err = allocation_value(:,3:5);        % Allocation error on three axes
u_alloc = allocation_value(:,12:15);  
n_event = length(event);
alloc_error = cell(n_event,1);
t_rel_alloc = cell(n_event,1);
u_alloc_event = cell(n_event,1);

for k = 1:n_event
    t0 = event(k) + 0.5 * step_time;
    t1 = t0 + Time - 1  * step_time;

    idx = (t_alloc >= t0) & (t_alloc <= t1);
    t_rel_alloc{k} = (t_alloc(idx) - t0) * 1e-6;  % Relative time (s)
    alloc_error{k} = alloc_err(idx,:);            % Allocation error (Nx3)
    u_alloc_event{k} = rad2deg(u_alloc(idx,:));
end

fig3 = figure(3);

% --- Roll allocation error ---
subplot(3,1,1); hold on; grid on;
plot(t_rel_alloc{1}, alloc_error{1}(:,1), '--', 'Color', inv_color, 'LineWidth', 0.5);
plot(t_rel_alloc{2}, alloc_error{2}(:,1), ':',  'Color', wls_color, 'LineWidth', 0.5);
plot(t_rel_alloc{3}, alloc_error{3}(:,1), '-.', 'Color', dir_color, 'LineWidth', 0.5);
plot(t_rel_alloc{4}, alloc_error{4}(:,1), '-',  'Color', pca_color, 'LineWidth', 0.5);

% title('Roll channel', 'FontSize', 8, 'FontName', 'Times New Roman');
xlabel('Time (s)');
% ylabel('$\epsilon_{r}$', 'Interpreter', 'latex');
ylabel('Roll channel');
axis([0 8 -20 10]); xticks(0:1:8);yticks(-20:5:10);
legend('INDI+INV','INDI+WLS','INDI+DIR','PINDI', ...
    'NumColumns', 2, 'Location', 'northeast', 'FontSize', 6, 'FontName', 'Times New Roman');

% --- Pitch allocation error ---
subplot(3,1,2); hold on; grid on;
plot(t_rel_alloc{1}, alloc_error{1}(:,2), '--', 'Color', inv_color, 'LineWidth', 0.5);
plot(t_rel_alloc{2}, alloc_error{2}(:,2), ':',  'Color', wls_color, 'LineWidth', 0.5);
plot(t_rel_alloc{3}, alloc_error{3}(:,2), '-.', 'Color', dir_color, 'LineWidth', 0.5);
plot(t_rel_alloc{4}, alloc_error{4}(:,2), '-',  'Color', pca_color, 'LineWidth', 0.5);

% title('Pitch channel', 'FontSize', 8, 'FontName', 'Times New Roman');
xlabel('Time (s)');
% ylabel('$\epsilon_{p}$', 'Interpreter', 'latex');
ylabel('Pitch channel');
axis([0 8 -5 15]); xticks(0:1:8);yticks(-5:5:15);

% --- Yaw allocation error ---
subplot(3,1,3); hold on; grid on;
plot(t_rel_alloc{1}, alloc_error{1}(:,3), '--', 'Color', inv_color, 'LineWidth', 0.5);
plot(t_rel_alloc{2}, alloc_error{2}(:,3), ':',  'Color', wls_color, 'LineWidth', 0.5);
plot(t_rel_alloc{3}, alloc_error{3}(:,3), '-.', 'Color', dir_color, 'LineWidth', 0.5);
plot(t_rel_alloc{4}, alloc_error{4}(:,3), '-',  'Color', pca_color, 'LineWidth', 0.5);

% title('Yaw channel', 'FontSize', 8, 'FontName', 'Times New Roman');
xlabel('Time (s)');
% ylabel('$\epsilon_{y}$', 'Interpreter', 'latex');
ylabel('Yaw channel');
axis([0 8 -30 30]); xticks(0:1:8);yticks(-30:10:30);

% sgtitle('Allocation error $\epsilon_{ca}$', 'Interpreter', 'latex', ...
%         'FontSize', 8, 'FontName', 'Times New Roman');

PlotToFileColorPDF(fig3,'results/Figure_21a.pdf',6.5,15.5); % flight_Alloc_error


%%
   fig4 = figure(4);

% --- u1  ---
subplot(4,1,1); hold on; grid on;
plot(t_rel_alloc{1}, u_alloc_event{1}(:,1), '--', 'Color', inv_color, 'LineWidth', 0.5);
plot(t_rel_alloc{2}, u_alloc_event{2}(:,1), ':',  'Color', wls_color, 'LineWidth', 0.5);
plot(t_rel_alloc{3}, u_alloc_event{3}(:,1), '-.', 'Color', dir_color, 'LineWidth', 0.5);
plot(t_rel_alloc{4}, u_alloc_event{4}(:,1), '-',  'Color', pca_color, 'LineWidth', 0.5);

% title('1 channel', 'FontSize', 8, 'FontName', 'Times New Roman');
xlabel('Time (s)');
ylabel('$u_1$ (deg)', 'Interpreter', 'latex');
axis([0 8 -20 20]); xticks(0:1:8);yticks(-20:10:20);
legend('INDI+INV','INDI+WLS','INDI+DIR','PINDI', ... %  'Location', 'southeast'
    'NumColumns', 2, 'FontSize', 6, 'FontName', 'Times New Roman','Location', 'southeast');

% --- u2  ---
subplot(4,1,2); hold on; grid on;
plot(t_rel_alloc{1}, u_alloc_event{1}(:,2), '--', 'Color', inv_color, 'LineWidth', 0.5);
plot(t_rel_alloc{2}, u_alloc_event{2}(:,2), ':',  'Color', wls_color, 'LineWidth', 0.5);
plot(t_rel_alloc{3}, u_alloc_event{3}(:,2), '-.', 'Color', dir_color, 'LineWidth', 0.5);
plot(t_rel_alloc{4}, u_alloc_event{4}(:,2), '-',  'Color', pca_color, 'LineWidth', 0.5);

% title('2 channel', 'FontSize', 8, 'FontName', 'Times New Roman');
xlabel('Time (s)');
ylabel('$u_2$ (deg)', 'Interpreter', 'latex');
axis([0 8 -20 20]); xticks(0:1:8);yticks(-20:10:20);

% --- u3  ---
subplot(4,1,3); hold on; grid on;
plot(t_rel_alloc{1}, u_alloc_event{1}(:,3), '--', 'Color', inv_color, 'LineWidth', 0.5);
plot(t_rel_alloc{2}, u_alloc_event{2}(:,3), ':',  'Color', wls_color, 'LineWidth', 0.5);
plot(t_rel_alloc{3}, u_alloc_event{3}(:,3), '-.', 'Color', dir_color, 'LineWidth', 0.5);
plot(t_rel_alloc{4}, u_alloc_event{4}(:,3), '-',  'Color', pca_color, 'LineWidth', 0.5);

% title('3 channel', 'FontSize', 8, 'FontName', 'Times New Roman');
xlabel('Time (s)');
ylabel('$u_3$ (deg)', 'Interpreter', 'latex');
axis([0 8 -20 20]); xticks(0:1:8);yticks(-20:10:20);

subplot(4,1,4); hold on; grid on;
plot(t_rel_alloc{1}, u_alloc_event{1}(:,4), '--', 'Color', inv_color, 'LineWidth', 0.5);
plot(t_rel_alloc{2}, u_alloc_event{2}(:,4), ':',  'Color', wls_color, 'LineWidth', 0.5);
plot(t_rel_alloc{3}, u_alloc_event{3}(:,4), '-.', 'Color', dir_color, 'LineWidth', 0.5);
plot(t_rel_alloc{4}, u_alloc_event{4}(:,4), '-',  'Color', pca_color, 'LineWidth', 0.5);

% title('4 channel', 'FontSize', 8, 'FontName', 'Times New Roman');
xlabel('Time (s)');
ylabel('$u_4$ (deg)', 'Interpreter', 'latex');
axis([0 8 -20 20]); xticks(0:1:8);yticks(-20:10:20);

% sgtitle('deflection', ...
        % 'FontSize', 8, 'FontName', 'Times New Roman');

PlotToFileColorPDF(fig4,'results/Figure_21b.pdf',6.5,15.5); % flight_Alloc_u
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

%% Position
% Time axis (μs)
t_set_pos   = vehicle_local_position_setpoint(:,1);
t_meas_pos  = vehicle_local_position(:,1);

% Position (m)
pos_set  = XYZ_setpoint;
pos_meas = XYZ;

% Initialize attitude error results
err_pos = cell(n_event,1);
t_rel_pos = cell(n_event,1);

% === Interpolation + Error Calculation: Attitude ===
for k = 1:n_event
    t0 = event(k) + 0.5 * step_time;
    t1 = t0 + Time - 1 * step_time;

    % Current window data (attitude)
    idx_set_pos  = (t_set_pos  >= t0) & (t_set_pos  <= t1);
    idx_meas_pos = (t_meas_pos >= t0) & (t_meas_pos <= t1);

    ts_set_pos  = (t_set_pos(idx_set_pos) - t0) * 1e-6;   % s
    ts_meas_pos = (t_meas_pos(idx_meas_pos) - t0) * 1e-6; % s

    d_set_pos  = pos_set(idx_set_pos,:);
    d_meas_pos = pos_meas(idx_meas_pos,:);

    % Interpolation & Error
    [err_pos{k}, t_rel_pos{k}] = interpolate_and_error(ts_set_pos, d_set_pos, ts_meas_pos, d_meas_pos);
end
fig5 = figure(5);



plot(err_pos{1}(:,1), err_pos{1}(:,2), '--', ...
    'Color', inv_color, 'LineWidth', 0.5);hold on;

plot(err_pos{2}(:,1), err_pos{2}(:,2), ':', ...
    'Color', wls_color, 'LineWidth', 0.5);hold on;

plot(err_pos{3}(:,1), err_pos{3}(:,2), '-.', ...
    'Color', dir_color, 'LineWidth', 0.5);hold on;

plot(err_pos{4}(:,1), err_pos{4}(:,2), '-', ...
    'Color', pca_color, 'LineWidth', 0.5);hold on;

% title('Trajectory in XY plane', 'FontSize', 8,'FontName','Times New Roman');
xlabel('X (m)');
ylabel('Y (m)');

legend('INDI+INV','INDI+WLS','INDI+DIR','PINDI', ...
       'Interpreter', 'latex', ...
       'Location', 'northwest', ...
       'NumColumns', 2,...
       'FontSize', 8);

grid on;
% axis([-1.5 1.5 -0.8 0.8]);  % 仅 XY
% xticks(-1.5:0.5:1.5);
% yticks(-0.8:0.4:0.8);

PlotToFileColorPDF(fig5, 'results/Figure_22.pdf', 7.5, 7.5); % flight_position_tracking
% PlotToFileColorPNG(fig5, 'results/flight_position_tracking.png', 20, 15);

% fig6 = figure(6);
% plot(XYZ_setpoint(:,1), XYZ_setpoint(:,2), '-', ...
%     'Color', inv_color, 'LineWidth', 0.5);hold on;
% 
% plot(XYZ(:,1), XYZ(:,2), '-', ...
%     'Color', wls_color, 'LineWidth', 1);hold on;


% for running times
disp('allocation running time (us), INV,WLS,DIR,PCA');
t_alloc = allocation_value(:,1);              % μs
running_time_alloc = allocation_value(:,2);              % μs
for k = 1:n_event
    t0 = event(k) + 0.5 * step_time;
    t1 = t0 + Time - 1  * step_time;

    % 
    idx_alloc  = (t_alloc  >= t0) & (t_alloc  <= t1);
    running_time_alloc_seg  = running_time_alloc(idx_alloc,:);
    mean(running_time_alloc_seg)

end

if ismember('rate_control_running_time', log.data.actuator_controls_0_0.Properties.VariableNames)
    disp('INDI running time (us)');
    mean(actuator_controls(actuator_controls(:,19)==1,18))
    disp('PID running time (us)');
    mean(actuator_controls(actuator_controls(:,19)==0,18))
end

