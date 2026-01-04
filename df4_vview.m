clear all;
close all;
addpath(genpath('~/Proj/control_allocation')); % Replace with the local path to your control_allocation(https://github.com/mengchaoheng/control_allocation) library.
%% 
set(groot, ...
    'defaultAxesFontSize', 8, ...
    'defaultAxesFontName', 'Times New Roman', ...
    'defaultAxesLineWidth', 0.5, ...
    'defaultAxesLabelFontSizeMultiplier', 1, ...
    'defaultAxesTitleFontSizeMultiplier', 1);
%%
l1=0.167;l2=0.069;k_v=3;
I_x=0.01149;
I_y=0.01153;
I_z=0.00487;
I=[I_x 0 0;0 I_y 0;0 0 I_z];

B=I\[-l1     0       l1     0;
     0      -l1     0       l1;
     l2    l2    l2    l2]*k_v;



%================================================================
[k,m] = size(B);
umin=ones(m,1)*(-20)*pi/180;
umax=ones(m,1)*20*pi/180;
plim=[umin umax];
fig1=figure,
% q=vview(B,plim,pinv(B))

q2=vview_case(B,plim,pinv(B))


axis([-40 40 -40 40 -60 60]);  % 仅 XY
xticks(-40:20:40);
yticks(-40:20:40);
zticks(-60:30:60);

PlotToFileColorPDF(fig1,'results/AMS_DF.pdf',7.5,7.5);
% PlotToFileColorPNG(fig1,'results/AMS_DF.png',20,20);