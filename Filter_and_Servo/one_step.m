close all;
% one step delta.  Amp=u_cmd = delta_u_cmd
T = 0.03;             % Time constant
delta_t=1/250;
delta_u_of_one_step= Amp* exp(-(delta_t ) / T)