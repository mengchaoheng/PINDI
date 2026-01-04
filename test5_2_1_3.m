clc; clear; close all;

% Data directory
data_dir = './data/test1_2_k_data';
files = dir(fullfile(data_dir, '*.mat'));

% Collect errors
all_errors = {};
for i = 1:length(files)
    load(fullfile(data_dir, files(i).name));

    % Time range: 200s to 700s (unit: μs)
    t_start = 150 * 1e6;
    t_end   = 250 * 1e6;

    % Extract and convert angular rate setpoint and measurement (unit: deg/s)
    t_set = log.data.vehicle_rates_setpoint_0{:,:}(:,1);
    pqr_set = rad2deg(log.data.vehicle_rates_setpoint_0{:,:}(:,2:4));

    t_meas = log.data.vehicle_angular_velocity_0{:,:}(:,1);
    pqr_meas = rad2deg(log.data.vehicle_angular_velocity_0{:,:}(:,3:5));

    % Crop time interval
    idx_set = (t_set >= t_start & t_set <= t_end);
    idx_meas = (t_meas >= t_start & t_meas <= t_end);

    t_set = t_set(idx_set);
    pqr_set = pqr_set(idx_set,:);

    t_meas = t_meas(idx_meas);
    pqr_meas = pqr_meas(idx_meas,:);

    % Determine which is longer, interpolate shorter data to the longer one
    if length(t_set) >= length(t_meas)
        pqr_set_interp = interp1(t_set, pqr_set, t_meas, 'linear', 'extrap');
        err = pqr_meas - pqr_set_interp;
    else
        pqr_meas_interp = interp1(t_meas, pqr_meas, t_set, 'linear', 'extrap');
        err = pqr_meas_interp - pqr_set;
    end

    % Compute error norm
    err_norm = sqrt(sum(err.^2, 2));
    all_errors{i} = err_norm;
end
set(groot, ...
    'defaultAxesFontSize', 8, ...
    'defaultAxesFontName', 'Times New Roman', ...
    'defaultAxesLineWidth', 0.5, ...
    'defaultAxesLabelFontSizeMultiplier', 1, ...
    'defaultAxesTitleFontSizeMultiplier', 1);

custom_labels = {'$\hat{k}_c=1$', '$\hat{k}_c=2$', '$\hat{k}_c=3$', '$\hat{k}_c=4$', '$\hat{k}_c=5$', '$\hat{k}_c=6$'};
fill_shades = [
    1.00 0.96 0.92;  
    0.98 0.88 0.78;   
    0.96 0.76 0.54;   
    0.93 0.60 0.30;   
    0.85 0.45 0.15;   
    0.70 0.30 0.05   
];

BoxMedianLineColor = [0.80, 0.10, 0.10];   
MarkerColor        = fill_shades(end, :);   
edge_color = [0.3, 0.1, 0.4]; 
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
        'BoxEdgeColor', 'k', ...  % Unified edge color 
        'BoxMedianLineColor', BoxMedianLineColor, ...  % Median line 
        'WhiskerLineColor', 'k', ...  % Whisker 
        'MarkerColor',MarkerColor, ...  % Outlier marker
        'MarkerStyle', '+', ...
        'MarkerSize', 4, ...
        'LineWidth', 0.5);
end
 
set(gca, ...
    'XTick', 1:6, ...
    'XTickLabel', custom_labels, ...
    'TickLabelInterpreter', 'latex');

ylabel('$\|\mathbf{e}(k)\|_2$ (deg/s)', 'Interpreter', 'latex');
% title('Angular velocity tracking error norm under different $\hat{k}_{c}$', 'Interpreter', 'latex', 'FontSize', 8);
grid on;
PlotToFileColorPDF(fig1,'results/Figure_13.pdf',10.5,4); % error_norm_k
% PlotToFileColorPNG(fig1,'results/error_norm_k.png',15,15);