clear all;
close all;
addpath(genpath('~/Proj/control_allocation')); % Replace with the local path to your control_allocation(https://github.com/mengchaoheng/control_allocation) library.
%%  
set(groot, ...
    'defaultAxesFontSize', 10, ...
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

figure,
q1=vview(B,plim,pinv(B))
figure,
vview(B,plim)
figure,
q2=vview_case(B,plim,pinv(B))
figure,
vview_case(B,plim)
%% 
 
B1= [1    0      -0.5;
     0      1     -0.5];

[k1,m1] = size(B1);
umin1=ones(m1,1)*-0.4;
umax1=ones(m1,1)*0.4;
plim1=[umin1 umax1];
figure,
q3=vview(B1,plim1,pinv(B1))
figure,
vview(B1,plim1)
figure,
q4=vview_case(B1,plim1,pinv(B1))
figure,
vview_case(B1,plim1)