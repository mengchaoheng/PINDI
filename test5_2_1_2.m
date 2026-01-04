clc;clear all; 
close all;
% Set data directory (can be replaced with an absolute path)
data_dir = './data/test1_2_time_const_data';  % or specify 'C:/Users/yourname/data/'

% Get all .mat files
files = dir(fullfile(data_dir, '*.mat'));

% Initialize error data list
all_errors = {};
labels = {};

% Iterate through each file
for i = 1:length(files)
    file_path = fullfile(data_dir, files(i).name);
    load(file_path);

    % ====== New error calculation: interpolation alignment ======
    t_start = 150 * 1e6;
    t_end   = 250 * 1e6;
    
    % Get time and data
    t_set = log.data.vehicle_rates_setpoint_0{:,:}(:,1);
    pqr_set = rad2deg(log.data.vehicle_rates_setpoint_0{:,:}(:,2:4));
    
    t_meas = log.data.vehicle_angular_velocity_0{:,:}(:,1);
    pqr_meas = rad2deg(log.data.vehicle_angular_velocity_0{:,:}(:,3:5));
    
    % Keep only data within the specified time range
    idx_set = (t_set >= t_start & t_set <= t_end);
    idx_meas = (t_meas >= t_start & t_meas <= t_end);
    
    t_set = t_set(idx_set);
    pqr_set = pqr_set(idx_set,:);
    
    t_meas = t_meas(idx_meas);
    pqr_meas = pqr_meas(idx_meas,:);
    
    % Determine which is longer, interpolate the shorter one
    if length(t_set) >= length(t_meas)
        % Interpolate measurement onto setpoint
        pqr_set_interp = interp1(t_set, pqr_set, t_meas, 'linear', 'extrap');
        err = pqr_meas - pqr_set_interp;
    else
        % Interpolate setpoint onto measurement
        pqr_meas_interp = interp1(t_meas, pqr_meas, t_set, 'linear', 'extrap');
        err = pqr_meas_interp - pqr_set;
    end
    
    % Compute error norm
    err_norm = sqrt(sum(err.^2, 2));

    % Store data
    all_errors{i} = err_norm;
end
set(groot, ...
    'defaultAxesFontSize', 8, ...
    'defaultAxesFontName', 'Times New Roman', ...
    'defaultAxesLineWidth', 0.5, ...
    'defaultAxesLabelFontSizeMultiplier', 1, ...
    'defaultAxesTitleFontSizeMultiplier', 1);

% Labels and colors
custom_labels = {'$\epsilon=0.008$', '$\epsilon=0.01$', '$\epsilon=0.03$', '$\epsilon=0.05$', '$\epsilon=0.07$', '$\epsilon=0.08$'}; % Custom x-axis labels

fill_shades = [   % (light to dark)
    0.90 0.95 1.00;
    0.75 0.87 0.98;
    0.60 0.78 0.95;
    0.45 0.68 0.90;
    0.30 0.55 0.85;
    0.15 0.40 0.75
];

BoxMedianLineColor = [0.80, 0.10, 0.10];     % Deep red to highlight the median line
% MarkerColor        = [0.30, 0.30, 0.50];   % Dark gray-blue to avoid distraction
MarkerColor        = fill_shades(end, :);     % The darkest fill color
edge_color = [0.3, 0.1, 0.4];  % Fixed dark purple edge color

% Concatenate data
data_vec = vertcat(all_errors{:});
group_vec = arrayfun(@(i) repmat(i, length(all_errors{i}), 1), ...
    1:length(all_errors), 'UniformOutput', false);
group_vec = vertcat(group_vec{:});

% Plot
fig1 = figure;
hold on;

for k = 1:6
    this_group = data_vec(group_vec == k);
    boxchart(ones(size(this_group)) * k, this_group, ...
        'BoxFaceColor', fill_shades(k,:), ...
        'BoxEdgeColor', 'k', ...  %  
        'BoxMedianLineColor', BoxMedianLineColor, ...  %  
        'WhiskerLineColor', 'k', ...  %  
        'MarkerColor',MarkerColor, ...  % 
        'MarkerStyle', '+', ...
        'MarkerSize', 4, ...
        'LineWidth', 0.5);
end
% Set axis
set(gca, ...
    'XTick', 1:6, ...
    'XTickLabel', custom_labels, ...
    'TickLabelInterpreter', 'latex');

ylabel('$\|\mathbf{e}(k)\|_2$ (deg/s)', 'Interpreter', 'latex');
custom_labels = {'$\epsilon=0.008$', '$\epsilon=0.01$', '$\epsilon=0.03$', '$\epsilon=0.05$', '$\epsilon=0.07$', '$\epsilon=0.09$'}; % 自定义为你想显示的x轴文字
% title('Angular velocity tracking error norm under different $\epsilon$', 'Interpreter', 'latex', 'FontSize', 8);
grid on;
PlotToFileColorPDF(fig1,'results/Figure_12.pdf',10.5,4); % error_norm_time_const
% PlotToFileColorPNG(fig1,'results/error_norm_time_const.png',15,15);