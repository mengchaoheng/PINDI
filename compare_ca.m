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
 
B= [1    0      0.5;
     0      -1     -0.5];

[k,m] = size(B);
umin=ones(m,1)*-0.5;
umax=ones(m,1)*0.5;
plim=[umin umax];

v_h=[0.7; 0];
v_c=[0.1; 0.38];
v=v_h+v_c;


% =========
u=pinv(B)*v;
u=min(max(u, umin), umax);
x_inv = u;%restoring(B,u,umin,umax);
v_inv= B*x_inv
% ===========
u =wls_alloc_gen(B,v,umin,umax,eye(k),eye(m),zeros(m,1),1e6,zeros(m,1),zeros(m,1),100,3);
u=min(max(u, umin), umax);
x_wls_gen = restoring(B,u,umin,umax);
v_wls=B*x_wls_gen;
% ==========
[u, ~,~] = DP_LPCA_prio([0; 0],v,B,umin,umax,100);
u=min(max(u, umin), umax);
x_dir =restoring_cpp(B,u,umin,umax);
v_dir= B*x_dir;
% ==========
[u, ~,~] = DP_LPCA_prio(v_h,v_c,B,umin,umax,100);
u=min(max(u, umin), umax);
x_PCA =restoring_cpp(B,u,umin,umax);
v_pca= B*x_PCA;
fig1=figure,

q=vview_compare(B,plim,pinv(B), v_h, v_c, v, v_inv,v_wls,v_dir,v_pca)

axis([-1.2 1.2 -1.5 0.9]);
xticks(-1.2:0.4:1.2);
yticks(-1.5:0.3:1.2);
% axis([-1 1 -1 1]);
% xticks(-1:0.5:1);
% yticks(-1:0.5:1);
PlotToFileColorPDF(fig1,'results/CA_compare.pdf',7.2,7.2); 
% PlotToFileColorPDF(fig1,'results/myplot.pdf',10,10); 



