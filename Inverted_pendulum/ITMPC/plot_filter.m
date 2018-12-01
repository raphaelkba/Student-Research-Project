clc 
clear all
close all

fname = 'SBMPC_inverted_pendulum_nofilter.mat';
nofilter = open(fname);
u_opt_nofilter = nofilter.u_opt;
states_history_nofilter = nofilter.states_history;
trajectory_history_nofilter = nofilter.trajectory_history ;
S_save_nofilter = nofilter.S_save;
param_nofilter = nofilter.param;

fname = 'SBMPC_inverted_pendulum_weightsfilter.mat';
weightsfilter = open(fname);
u_opt_weightsfilter = weightsfilter.u_opt;
states_history_weightsfilter = weightsfilter.states_history;
trajectory_history_weightsfilter = weightsfilter.trajectory_history ;
S_save_weightsfilter = weightsfilter.S_save;
param_weightsfilter = weightsfilter.param;

fname = 'SBMPC_inverted_pendulum_controlfilter.mat';
filtercontrol = open(fname);
u_opt_filtercontrol = filtercontrol.u_opt;
states_history_filtercontrol = filtercontrol.states_history;
trajectory_history_filtercontrol = filtercontrol.trajectory_history ;
S_save_filtercontrol = filtercontrol.S_save;
param_filtercontrol = filtercontrol.param;

fname = 'SBMPC_inverted_pendulum_inputopt_noise.mat';
inputopt = open(fname);
u_opt_inputopt = inputopt.u_opt;
states_history_inputopt = inputopt.states_history;
trajectory_history_inputopt = inputopt.trajectory_history ;
S_save_inputopt = inputopt.S_save;
param_inputopt = inputopt.param;

figure
t = linspace(0,5,size(u_opt_nofilter,1));
plot(t, u_opt_nofilter(:),'r');
hold on
t = linspace(0,5,size( u_opt_weightsfilter,1));
plot(t, u_opt_weightsfilter,'b');
hold on
t = linspace(0,5,size( u_opt_filtercontrol,1));
plot(t, u_opt_filtercontrol,'k');

itm_formatfig('LatexWide')

xlabel('$t [s]');
ylabel('u [N]');
legend({'no filter','filter on the control','filter on the weights'})

print('-depsc', 'filter_comparison.eps')
print('-dpng', 'filter_comparison.png')