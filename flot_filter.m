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
ulgFileName = 'data/log_54_2026-7-25-15-48-54';
channel = 1; % control surface 1-4
time_range = []; % []: whole log; or [start end] seconds from log start
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

% In PINDI, see pipeline_of_u.txt

if ~isfield(log.data, 'allocation_value_0') || ...
        ~isfield(log.data, 'actuator_outputs_value_0')
    error('The log must contain allocation_value and actuator_outputs_value.');
end

allocation = log.data.allocation_value_0;
feedback = log.data.actuator_outputs_value_0;
field_index = channel - 1;

% Read by ULog field name. Added/reordered message columns do not affect this.
t_allocation = topic_field(allocation, 'timestamp') * 1e-6;
u = topic_field(allocation, sprintf('u[%d]', field_index));
u_ultimate = topic_field(allocation, ...
    sprintf('u_ultimate[%d]', field_index));

t_feedback = topic_field(feedback, 'timestamp') * 1e-6;
delta = topic_field(feedback, sprintf('delta[%d]', field_index));

t0 = min(t_allocation(1), t_feedback(1));
t_allocation = t_allocation - t0;
t_feedback = t_feedback - t0;

estimate_field = sprintf('estimate[%d]', field_index);
has_estimate = topic_has_field(feedback, estimate_field);

figure('Name', 'Logged INDI actuator chain', 'Color', 'w');
plot_handles = gobjects(0);
plot_labels = {};

plot_handles(end+1) = plot(t_allocation, u, '-', ...
    'Color', [0.65 0.65 0.65], ...
    'LineWidth', 0.7);
plot_labels{end+1} = 'Allocation u';
hold on;
plot_handles(end+1) = plot(t_allocation, u_ultimate, ...
    'k--', 'LineWidth', 0.9);
plot_labels{end+1} = 'u ultimate';

if has_estimate
    estimate = topic_field(feedback, estimate_field);
    plot_handles(end+1) = plot(t_feedback, estimate, ...
        'b-', 'LineWidth', 0.9);
    plot_labels{end+1} = 'Actuator estimate';
else
    warning(['The selected log has no "%s" field. ' ...
        'The actuator-estimate curve is omitted.'], estimate_field);
end

plot_handles(end+1) = plot(t_feedback, delta, ...
    'r-', 'LineWidth', 0.9);
plot_labels{end+1} = 'Filtered delta (INDI feedback)';
grid on;
if ~isempty(time_range)
    xlim(time_range);
end
xlabel('Time (s)');
ylabel('Deflection (rad)');
title(sprintf('Surface %d: allocation -> actuator -> low-pass', channel));
legend(plot_handles, plot_labels, 'Location', 'best');

function present = topic_has_field(topic_table, ulog_field_name)
descriptions = topic_table.Properties.VariableDescriptions;
valid_name = matlab.lang.makeValidName(ulog_field_name);
present = any(strcmp(descriptions, ulog_field_name)) || ...
    any(strcmp(topic_table.Properties.VariableNames, valid_name));
end

function value = topic_field(topic_table, ulog_field_name)
% Read a field by its original ULog name, independent of column order.
descriptions = topic_table.Properties.VariableDescriptions;
index = find(strcmp(descriptions, ulog_field_name), 1);

if isempty(index)
    valid_name = matlab.lang.makeValidName(ulog_field_name);
    index = find(strcmp(topic_table.Properties.VariableNames, valid_name), 1);
end

if isempty(index)
    error('Field "%s" was not found in topic.', ulog_field_name);
end

value = double(topic_table{:, index});
end
