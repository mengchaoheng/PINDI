% test2_rms_segmented.m
clc; clear; close all;

% Set data directory
base_dir = './data/test2_alloc_data'; 
files = dir(fullfile(base_dir, '*.mat'));

% Initialize RMS error sets
all_rms_rates = [];
all_rms_att   = [];
group_ids = [];
labels = {};

for i = 1:length(files)
    file_path = fullfile(base_dir, files(i).name);
    load(file_path);  % Load log struct

    % ====== Read raw angular rate data ======
    rates_sp_raw = log.data.vehicle_rates_setpoint_0{:,:};
    rates_meas_raw = log.data.vehicle_angular_velocity_0{:,:};
    t_rates_sp = rates_sp_raw(:,1);
    t_rates_meas = rates_meas_raw(:,1);
    rates_sp = rad2deg(rates_sp_raw(:,2:4));
    rates_meas = rad2deg(rates_meas_raw(:,3:5));

    % ====== Attitude response: quaternion to Euler angles ======
    attitude_meas_raw = log.data.vehicle_attitude_0{:,:};
    t_att_meas = attitude_meas_raw(:,1);
    q_0 = attitude_meas_raw(:,3);
    q_1 = attitude_meas_raw(:,4);
    q_2 = attitude_meas_raw(:,5);
    q_3 = attitude_meas_raw(:,6);
    Roll  = rad2deg(quat_to_roll([q_0 q_1 q_2 q_3]));
    Pitch = rad2deg(quat_to_pitch([q_0 q_1 q_2 q_3]));
    Yaw   = rad2deg(quat_to_yaw([q_0 q_1 q_2 q_3]));
    att_meas = [Roll, Pitch, Yaw];

    % ====== Attitude command: quaternion to Euler angles ======
    att_sp_raw = log.data.vehicle_attitude_setpoint_0{:,:};
    t_att_sp = att_sp_raw(:,1);
    q_0_sp = att_sp_raw(:,6);
    q_1_sp = att_sp_raw(:,7);
    q_2_sp = att_sp_raw(:,8);
    q_3_sp = att_sp_raw(:,9);
    Roll_sp  = rad2deg(quat_to_roll([q_0_sp q_1_sp q_2_sp q_3_sp]));
    Pitch_sp = rad2deg(quat_to_pitch([q_0_sp q_1_sp q_2_sp q_3_sp]));
    Yaw_sp   = rad2deg(quat_to_yaw([q_0_sp q_1_sp q_2_sp q_3_sp]));
    att_sp = [Roll_sp, Pitch_sp, Yaw_sp];

    % ====== Command information ======
    cmd = log.data.vehicle_command_0{:,:};
    parameter_update=log.data.parameter_update_0{:,:};
    t_start = parameter_update(3);
    t_end = parameter_update(4);
    idx_ref = find(cmd(:,1) > t_start & cmd(:,1) < t_end);
    t_cmd = cmd(idx_ref,1);
    flag_cmd = cmd(idx_ref,16);

    % Find all timestamps for modes 2 and 3
    t_mode2 = t_cmd(flag_cmd == 1);
    t_mode3 = t_cmd(flag_cmd == 2);

    % Construct paired intervals (mode2 -> next mode3)
    pairs = [];
    for k = 1:length(t_mode2)
        s_time = t_mode2(k);
        e_time = t_mode3(find(t_mode3 > s_time, 1));
        if ~isempty(e_time)
            pairs = [pairs; s_time, e_time];
        end
    end

    % ====== Iterate over each interval, interpolate and compute RMS ======
    rms_rates_each = [];
    rms_att_each = [];

    for j = 1:size(pairs,1)
        s_time = pairs(j,1);
        e_time = pairs(j,2);
    
        % ===== Angular rate error =====
        idx_sp_rates   = (t_rates_sp   >= s_time & t_rates_sp   <= e_time);
        idx_meas_rates = (t_rates_meas >= s_time & t_rates_meas <= e_time);
    
        t1_rates = t_rates_sp(idx_sp_rates);
        d1_rates = rates_sp(idx_sp_rates,:);
        t2_rates = t_rates_meas(idx_meas_rates);
        d2_rates = rates_meas(idx_meas_rates,:);
    
        if length(t1_rates) < 2 || length(t2_rates) < 2
            continue;
        end
        err_rates = interpolate_and_error(t1_rates, d1_rates, t2_rates, d2_rates);
    
        % ===== Attitude error =====
        idx_sp_att   = (t_att_sp   >= s_time & t_att_sp   <= e_time);
        idx_meas_att = (t_att_meas >= s_time & t_att_meas <= e_time);
    
        t1_att = t_att_sp(idx_sp_att);
        d1_att = att_sp(idx_sp_att,:);
        t2_att = t_att_meas(idx_meas_att);
        d2_att = att_meas(idx_meas_att,:);
    
        if length(t1_att) < 2 || length(t2_att) < 2
            continue;
        end
        [err_att,~] = interpolate_and_error(t1_att, d1_att, t2_att, d2_att);
    
        % ===== Compute RMS and aggregate =====
        rms_rates = sqrt(mean(err_rates.^2, 1));
        rms_att   = sqrt(mean(err_att.^2, 1));
    
        rms_rates_each = [rms_rates_each; rms_rates];
        rms_att_each   = [rms_att_each;   rms_att];
    end

    % Aggregate
    all_rms_rates = [all_rms_rates; rms_rates_each];
    all_rms_att   = [all_rms_att; rms_att_each];
    group_ids = [group_ids; repmat(i, size(rms_rates_each,1), 1)];
    labels{end+1} = ['File ', num2str(i)];
end
%% 
set(groot, ...
    'defaultAxesFontSize', 8, ...
    'defaultAxesFontName', 'Times New Roman', ...
    'defaultAxesLineWidth', 0.5, ...
    'defaultAxesLabelFontSizeMultiplier', 1, ...
    'defaultAxesTitleFontSizeMultiplier', 1);
%% Combined figure
fig1 = figure(1);
custom_labels = {'INDI+INV','INDI+WLS','INDI+DIR','PINDI'};

% Color definitions
rate_names = {'Roll','Pitch','Yaw'};
rate_color = [0.15 0.40 0.75];     % Blue - angular rate
att_color  = [0.70 0.30 0.05];     % Brown-red - attitude angle
% Set custom y-axis ranges (adjust as needed)

rate_ylim = {[3.3 10.2], [3.4 11.2], [22.5 31]};   % Right y-axis, angular rate deg/s
att_ylim  = {[0 1.4], [0 1.8], [19.1 21.8]};       % Left y-axis, attitude angle deg
for i = 1:3
    subplot(1,3,i);

    % === Left axis: attitude angle ===
    yyaxis left
    ax = gca;  
    ax.YColor = att_color;  % Set left axis color to red
    if i==3
        ax.YTick=16:1:20;
    end
    hold on;
    for j = 1:4
        idx = (group_ids == j);
        h1 =boxchart(ones(sum(idx),1)*(j-0.15), all_rms_att(idx,i), ...
            'BoxFaceColor', att_color, ...
            'BoxEdgeColor', 'k', ...
            'BoxWidth', 0.25, ...
            'LineWidth', 0.8, ...
            'MarkerStyle', '+', ...
            'MarkerColor', att_color, ...
            'MarkerSize', 4);
    end
    ylabel([rate_names{i} ' Angle (deg)']);
    % ylim(att_ylim{i});

    % === Right axis: angular rate ===
    yyaxis right
    ax = gca;
    ax.YColor = rate_color;  % Set right axis color to blue
    hold on;
    for j = 1:4
        idx = (group_ids == j);
        h2 =boxchart(ones(sum(idx),1)*(j+0.15), all_rms_rates(idx,i), ...
            'BoxFaceColor', rate_color, ...
            'BoxEdgeColor', 'k', ...
            'BoxWidth', 0.25, ...
            'LineWidth', 0.8, ...
            'MarkerStyle', 'x', ...
            'MarkerColor', rate_color, ...
            'MarkerSize', 4);
    end
    ylabel([rate_names{i} ' Rate (deg/s)']);
    % ylim(rate_ylim{i});

    set(gca, ...
        'XTick', 1:4, ...
        'XTickLabel', custom_labels);
    xlim([0.5 4.5]);
    grid on;
end

legend([h1 h2], {'Angle', 'Rate'}, ...
            'Position',[0.630452799676572 0.939530676786843 0.229411764705882 0.0549019607843136],...
    'NumColumns',2,...
    'Interpreter','none',...
    'FontSize',8);

% === Overall title ===
% sgtitle('Tracking RMS Error Comparison Across Roll, Pitch, and Yaw Channels', 'FontSize', 8,'FontName','Times New Roman');

PlotToFileColorPDF(fig1,'results/Figure_17.pdf',16,8.5); % tracking_RMS_error

% === Save data for statistical analysis ===
if ~exist('results','dir'), mkdir results; end
save('results/rms_data.mat', ...
     'all_rms_rates','all_rms_att','group_ids','custom_labels');
fprintf('Analysis data saved to results/rms_data.mat\n');