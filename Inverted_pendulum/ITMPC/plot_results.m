%% Plot results
% sorry for the confused code
% each .mat file contains the necessary data of different simulations
clc 
clear all
close all

fname = 'dt001nmpc.mat';
NMPC = open(fname);
fname = 'SBMPC_inverted_pendulum_1.mat';
data_1 = open(fname);
fname = 'SBMPC_inverted_pendulum_2.mat';
data_2 = open(fname);
fname = 'SBMPC_inverted_pendulum_3.mat';
data_3 = open(fname);
fname = 'SBMPC_inverted_pendulum_4.mat';
data_4 = open(fname);
fname = 'SBMPC_inverted_pendulum_5.mat';
data_5 = open(fname);
fname = 'SBMPC_inverted_pendulum_6.mat';
data_6 = open(fname);
fname = 'SBMPC_inverted_pendulum_7.mat';
data_7 = open(fname);
fname = 'SBMPC_inverted_pendulum_8.mat';
data_8 = open(fname);
fname = 'SBMPC_inverted_pendulum_9.mat';
data_9 = open(fname);
fname = 'SBMPC_inverted_pendulum_10.mat';
data_10 = open(fname);
fname = 'SBMPC_inverted_pendulum_samples2000noise2.mat';
data_noisy = open(fname);

fname = 'SBMPC_inverted_pendulum_alpha02_1.mat';
data_alpha = open(fname);

fname = 'SBMPC_inverted_pendulum_dt_1.mat';
data_dt_1 = open(fname);
fname = 'SBMPC_inverted_pendulum_dt_2.mat';
data_dt_2 = open(fname);
fname = 'SBMPC_inverted_pendulum_dt_3_nodecay.mat';
data_dt_3 = open(fname);
fname = 'SBMPC_inverted_pendulum_dt_4_nodecay.mat';
data_dt_4 = open(fname);
fname = 'SBMPC_inverted_pendulum_dt_5_nodecay.mat';
data_dt_5 = open(fname);
fname = 'SBMPC_inverted_pendulum_dt_6_nodecay.mat';
data_dt_6 = open(fname);
fname = 'SBMPC_inverted_pendulum_dt_7_nodecay.mat';
data_dt_7 = open(fname);
fname = 'SBMPC_inverted_pendulum_dt_8_nodecay.mat';
data_dt_8 = open(fname);
fname = 'SBMPC_inverted_pendulum_dt_9_nodecay.mat';
data_dt_9 = open(fname);
fname = 'SBMPC_inverted_pendulum_dt_10_nodecay.mat';
data_dt_10 = open(fname);

u_opt_dt_1 = data_dt_1.u_opt;
states_history_dt_1 = data_dt_1.states_history;
trajectory_history_dt_1 = data_dt_1.trajectory_history ;
S_save_dt_1 = data_dt_1.S_save;
param_dt_1 = data_dt_1.param;

u_opt_dt_2 = data_dt_2.u_opt;
states_history_dt_2 = data_dt_2.states_history;
trajectory_history_dt_2 = data_dt_2.trajectory_history ;
S_save_dt_2 = data_dt_2.S_save;
param_dt_2 = data_dt_2.param;

u_opt_dt_3 = data_dt_3.u_opt;
states_history_dt_3 = data_dt_3.states_history;
trajectory_history_dt_3 = data_dt_3.trajectory_history ;
S_save_dt_3 = data_dt_3.S_save;
param_dt_3 = data_dt_3.param;

u_opt_dt_4 = data_dt_4.u_opt;
states_history_dt_4 = data_dt_4.states_history;
trajectory_history_dt_4 = data_dt_4.trajectory_history ;
S_save_dt_4 = data_dt_4.S_save;
param_dt_4 = data_dt_4.param;

u_opt_dt_5 = data_dt_5.u_opt;
states_history_dt_5 = data_dt_5.states_history;
trajectory_history_dt_5 = data_dt_5.trajectory_history ;
S_save_dt_5 = data_dt_5.S_save;
param_dt_5 = data_dt_5.param;

u_opt_dt_6 = data_dt_6.u_opt;
states_history_dt_6 = data_dt_6.states_history;
trajectory_history_dt_6 = data_dt_6.trajectory_history ;
S_save_dt_6 = data_dt_6.S_save;
param_dt_6 = data_dt_6.param;

u_opt_dt_7 = data_dt_7.u_opt;
states_history_dt_7 = data_dt_7.states_history;
trajectory_history_dt_7 = data_dt_7.trajectory_history ;
S_save_dt_7 = data_dt_7.S_save;
param_dt_7 = data_dt_7.param;

u_opt_dt_8 = data_dt_8.u_opt;
states_history_dt_8 = data_dt_8.states_history;
trajectory_history_dt_8 = data_dt_8.trajectory_history ;
S_save_dt_8 = data_dt_8.S_save;
param_dt_8 = data_dt_8.param;

u_opt_dt_9 = data_dt_9.u_opt;
states_history_dt_9 = data_dt_9.states_history;
trajectory_history_dt_9 = data_dt_9.trajectory_history ;
S_save_dt_9 = data_dt_9.S_save;
param_dt_9 = data_dt_9.param;


u_opt_dt_10 = data_dt_10.u_opt;
states_history_dt_10 = data_dt_10.states_history;
trajectory_history_dt_10 = data_dt_10.trajectory_history ;
S_save_dt_10 = data_dt_10.S_save;
param_dt_10 = data_dt_10.param;



u_opt_alpha = data_alpha.u_opt;
states_history_alpha = data_alpha.states_history;
trajectory_history_alpha = data_alpha.trajectory_history ;
S_save_alpha = data_alpha.S_save;
param_alpha = data_alpha.param;

fname = 'SBMPC_inverted_pendulum_dt_1.mat';
data_dt_small = open(fname);

u_opt_dt_small = data_dt_small.u_opt;
states_history_dt_small = data_dt_small.states_history;
trajectory_history_dt_small = data_dt_small.trajectory_history ;
S_save_dt_small = data_dt_small.S_save;
param_dt_small = data_dt_small.param;

fname = 'SBMPC_inverted_pendulum_horizon_filter_0.01.mat';
data_nofilter = open(fname);

u_opt_nofilte = data_nofilter.u_opt;
states_history_nofilte = data_nofilter.states_history;
trajectory_history_nofilte = data_nofilter.trajectory_history ;
S_save_nofilte = data_nofilter.S_save;
param_nofilte = data_nofilter.param;

fname = 'SBMPC_inverted_pendulum_horizon_filtercontrol.mat';
data_filtercontrol = open(fname);

u_opt_filtercontrol = data_filtercontrol.u_opt;
states_history_filtercontrol = data_filtercontrol.states_history;
trajectory_history_filtercontrol = data_filtercontrol.trajectory_history ;
S_save_filtercontrol = data_filtercontrol.S_save;
param_filtercontrol = data_filtercontrol.param;



fname = 'SBMPC_inverted_pendulum_horizon_filter.mat';
data_filter_2 = open(fname);

u_opt_filter_2 = data_filter_2.u_opt;
states_history_filter_2 = data_filter_2.states_history;
trajectory_history_filter_2 = data_filter_2.trajectory_history ;
S_save_filter_2 = data_filter_2.S_save;
param_filter_2 = data_filter_2.param;



fname = 'SBMPC_inverted_pendulum_horizon15.mat';
data_horizon = open(fname);

u_opt_horizon = data_horizon.u_opt;
states_history_horizon = data_horizon.states_history;
trajectory_history_horizon = data_horizon.trajectory_history ;
S_save_horizon = data_horizon.S_save;
param_horizon = data_horizon.param;


fname = 'SBMPC_inverted_pendulum_lambda_5.mat';
data_lambda = open(fname);

u_opt_lambda = data_lambda.u_opt;
states_history_lambda = data_lambda.states_history;
trajectory_history_lambda = data_lambda.trajectory_history ;
S_save_lambda = data_lambda.S_save;
param_lambda = data_lambda.param;

fname = 'SBMPC_inverted_pendulum_noisedecay.mat';
data_noisedecay = open(fname);

u_opt_noisedecay = data_noisedecay.u_opt;
states_history_noisedecay = data_noisedecay.states_history;
trajectory_history_noisedecay = data_noisedecay.trajectory_history ;
S_save_noisedecay = data_noisedecay.S_save;
param_noisedecay = data_noisedecay.param;

fname = 'SBMPC_inverted_pendulum_samples2000.mat';
data_samples = open(fname);

u_opt_samples = data_samples.u_opt;
states_history_samples = data_samples.states_history;
trajectory_history_samples = data_samples.trajectory_history ;
S_save_samples = data_samples.S_save;
param_samples = data_samples.param;


u_opt_1 = data_1.u_opt;
states_history_1 = data_1.states_history;
trajectory_history_1 = data_1.trajectory_history ;
S_save_1 = data_1.S_save;
param_1 = data_1.param;

u_opt_2 = data_2.u_opt;
states_history_2 = data_2.states_history;
trajectory_history_2 = data_2.trajectory_history ;
S_save_2 = data_2.S_save;
param_2 = data_2.param;

u_opt_3 = data_3.u_opt;
states_history_3 = data_3.states_history;
trajectory_history_3 = data_3.trajectory_history ;
S_save_3 = data_3.S_save;
param_3 = data_3.param;

u_opt_4 = data_4.u_opt;
states_history_4 = data_4.states_history;
trajectory_history_4 = data_4.trajectory_history ;
S_save_4 = data_4.S_save;
param_4 = data_4.param;

u_opt_5 = data_5.u_opt;
states_history_5 = data_5.states_history;
trajectory_history_5 = data_5.trajectory_history ;
S_save_5 = data_5.S_save;
param_5 = data_5.param;

u_opt_6 = data_6.u_opt;
states_history_6 = data_6.states_history;
trajectory_history_6 = data_6.trajectory_history ;
S_save_6 = data_6.S_save;
param_6 = data_6.param;

u_opt_7 = data_7.u_opt;
states_history_7 = data_7.states_history;
trajectory_history_7 = data_7.trajectory_history ;
S_save_7 = data_7.S_save;
param_7 = data_7.param;

u_opt_8 = data_8.u_opt;
states_history_8 = data_8.states_history;
trajectory_history_8 = data_8.trajectory_history ;
S_save_8 = data_8.S_save;
param_8 = data_8.param;

u_opt_9 = data_9.u_opt;
states_history_9 = data_9.states_history;
trajectory_history_9 = data_9.trajectory_history ;
S_save_9 = data_9.S_save;
param_9 = data_9.param;

u_opt_10 = data_10.u_opt;
states_history_10 = data_10.states_history;
trajectory_history_10 = data_10.trajectory_history ;
S_save_10 = data_10.S_save;
param_10 = data_10.param;

u_opt_noisy = data_noisy.u_opt;
states_history_noisy = data_noisy.states_history;
trajectory_history_noisy = data_noisy.trajectory_history ;
S_save_noisy = data_noisy.S_save;
param_noisy = data_noisy.param;

fname = 'SBMPC_inverted_pendulum_test_initialinput.mat';
fname = 'SBMPC_inverted_pendulum_inputopt_noise.mat';

intial_input = open(fname);

u_opt_intial_input = intial_input.u_opt;
states_history_intial_input = intial_input.states_history;
trajectory_history_intial_input = intial_input.trajectory_history ;
S_save_intial_input = intial_input.S_save;
param_intial_input = intial_input.param;


u_NMPC = NMPC.u;
x_NMPC = NMPC.x;

% figure(1)

% for i=1:1:size(states_history_1,1)
%     
%    x_std(i) = std([states_history_1(i,1);states_history_2(i,1);states_history_3(i,1);...
%        states_history_4(i,1);states_history_5(i,1);states_history_6(i,1);...
%        states_history_7(i,1);]);
%    y_std(i) = std([states_history_1(i,2);states_history_2(i,2);states_history_3(i,2);...
%        states_history_4(i,2);states_history_5(i,2);states_history_6(i,2);...
%        states_history_7(i,2);]);
%    x_mean(i,:) = mean([states_history_1(i,:);states_history_2(i,:);states_history_3(i,:);...
%        states_history_4(i,:);states_history_5(i,:);states_history_6(i,:);...
%        states_history_7(i,:);]);
%    y_mean(i) = mean([states_history_1(i,2);states_history_2(i,2);states_history_3(i,2);...
%        states_history_4(i,2);states_history_5(i,2);states_history_6(i,2);...
%        states_history_7(i,2);]);
% 
% end

% plot(x_mean(:,1) + sin(x_mean(:,3))*0.5,0.2+ 0.05 + cos(x_mean(:,3))*0.5)


% figure(2)
% plot(states_history_1(:,1) + sin(states_history_1(:,3))*0.5,0.2+ 0.05 + cos(states_history_1(:,3))*0.5)
% hold on    
% plot(states_history_2(:,1) + sin(states_history_2(:,3))*0.5,0.2+ 0.05 + cos(states_history_2(:,3))*0.5)
% plot(states_history_3(:,1) + sin(states_history_3(:,3))*0.5,0.2+ 0.05 + cos(states_history_3(:,3))*0.5)
% plot(states_history_4(:,1) + sin(states_history_4(:,3))*0.5,0.2+ 0.05 + cos(states_history_4(:,3))*0.5)
% plot(states_history_5(:,1) + sin(states_history_5(:,3))*0.5,0.2+ 0.05 + cos(states_history_5(:,3))*0.5)
% plot(states_history_6(:,1) + sin(states_history_6(:,3))*0.5,0.2+ 0.05 + cos(states_history_6(:,3))*0.5)
% plot(states_history_7(:,1) + sin(states_history_7(:,3))*0.5,0.2+ 0.05 + cos(states_history_7(:,3))*0.5)
% plot(states_history_8(:,1) + sin(states_history_8(:,3))*0.5,0.2+ 0.05 + cos(states_history_8(:,3))*0.5)
% plot(states_history_9(:,1) + sin(states_history_9(:,3))*0.5,0.2+ 0.05 + cos(states_history_9(:,3))*0.5)
% plot(states_history_10(:,1) + sin(states_history_10(:,3))*0.5,0.2+ 0.05 + cos(states_history_10(:,3))*0.5)
% 
% itm_formatfig('LatexWide')
% title(['Pendulum trajectories for different SBMPC simulations'],'Interpreter','latex');
% xlabel('$x$ [m]','Interpreter','latex');
% ylabel('$y$ [m]','Interpreter','latex');
% legend({'Simulation 1','Simulation 2','Simulation 3','Simulation 4','Simulation 5', ...
%     'Simulation 6','Simulation 7','Simulation 8','Simulation 9','Simulation 10',},'Interpreter','latex')
% 
% print('-depsc', 'Pendulum_trajectory_det.eps')
% print('-dpng', 'Pendulum_trajectory_det.png')
% 
% 
% 
% 
t = linspace(0,50,size(states_history_1,1))/10;
% t_NMPC = linspace(0,50,size(x_NMPC,1))/10;
% % for i = 1:1:4
f=figure(3);
% p = uipanel('Parent',f); 
% p.Title = 'Pendulum on a Cart States - SBMPC simulations'; 
% p.TitlePosition = 'centertop'; 
% p.FontSize = 12;
% p.FontWeight = 'bold';

i=1;
ax1 = subplot(2,2,1);
plot(t,states_history_1(:,i))
hold on
plot(t,states_history_2(:,i))
plot(t,states_history_3(:,i))
plot(t,states_history_4(:,i))
plot(t,states_history_5(:,i))
plot(t,states_history_6(:,i))
plot(t,states_history_7(:,i))
plot(t,states_history_8(:,i))
plot(t,states_history_9(:,i))
plot(t,states_history_10(:,i))
set(gca,'XTick',0:1:5)
% plot(t,states_history_10(:,i))
% plot(t_NMPC,x_NMPC(:,i),'*r')
% plot(t,states_history_noisy(:,i),'*b')
xlabel('t [s]');
ylabel('x [m]');
axis tight
% legend({'Simulation 1','Simulation 2','Simulation 3','Simulation 4','Simulation 5', ...
%     'Simulation 6','Simulation 7','Simulation 8','Simulation 9','Simulation 10',},'Interpreter','latex')

i=2;
ax2 = subplot(2,2,2);
plot(t,states_history_1(:,i))
hold on
plot(t,states_history_2(:,i))
plot(t,states_history_3(:,i))
plot(t,states_history_4(:,i))
plot(t,states_history_5(:,i))
plot(t,states_history_6(:,i))
plot(t,states_history_7(:,i))
plot(t,states_history_8(:,i))
plot(t,states_history_9(:,i))
plot(t,states_history_10(:,i))
set(gca,'XTick',0:1:5)
% plot(t,states_history_10(:,i))
% plot(t_NMPC,x_NMPC(:,i),'*r')
% plot(t,states_history_noisy(:,i),'*b')
xlabel('t [s]');
ylabel('dot [m]');
% legend({'Simulation 1','Simulation 2','Simulation 3','Simulation 4','Simulation 5', ...
%     'Simulation 6','Simulation 7','Simulation 8','Simulation 9','Simulation 10',},'Interpreter','latex')
axis tight

i=3;
ax3 = subplot(2,2,3);
plot(t,states_history_1(:,i))
hold on
plot(t,states_history_2(:,i))
plot(t,states_history_3(:,i))
plot(t,states_history_4(:,i))
plot(t,states_history_5(:,i))
plot(t,states_history_6(:,i))
plot(t,states_history_7(:,i))
plot(t,states_history_8(:,i))
plot(t,states_history_9(:,i))
plot(t,states_history_10(:,i))
set(gca,'XTick',0:1:5)
% plot(t,states_history_10(:,i))
% plot(t_NMPC,x_NMPC(:,i),'*r')
% plot(t,states_history_noisy(:,i),'*b')
xlabel('t [s]');
ylabel('theta [rad]');
% legend({'Simulation 1','Simulation 2','Simulation 3','Simulation 4','Simulation 5', ...
%     'Simulation 6','Simulation 7','Simulation 8','Simulation 9','Simulation 10',},'Interpreter','latex')
axis tight

i=4;
ax4 = subplot(2,2,4);
plot(t,states_history_1(:,i))
hold on
plot(t,states_history_2(:,i))
plot(t,states_history_3(:,i))
plot(t,states_history_4(:,i))
plot(t,states_history_5(:,i))
plot(t,states_history_6(:,i))
plot(t,states_history_7(:,i))
plot(t,states_history_8(:,i))
plot(t,states_history_9(:,i))
plot(t,states_history_10(:,i))
set(gca,'XTick',0:1:5)
% plot(t,states_history_10(:,i))
% plot(t_NMPC,x_NMPC(:,i),'*r')
% plot(t,states_history_noisy(:,i),'*b')
xlabel('t [s]');
ylabel('dot');
% legend({'Simulation 1','Simulation 2','Simulation 3','Simulation 4','Simulation 5', ...
%     'Simulation 6','Simulation 7','Simulation 8','Simulation 9','Simulation 10',},'Interpreter','latex')
axis tight



itm_formatfig('LatexWide')
% % title(['Pendulum on a Cart States - SBMPC simulations'],'Interpreter','latex');
% 
% legend({'Sim 1','Sim 2','Sim 3','Sim 4','Sim 5', ...
%     'Sim 6','Sim 7','Sim 8','Sim 9','Sim 10',},'Interpreter','latex')
% print('-depsc', 'Pendulum_states_det.eps')
print('-dpng', 'Pendulum_states_det.png')

% 
% 
% 
% 
% Q = diag([100,1,100,1]); % Weight matrix (states)
% for i=1:1:size(states_history_1,1)
%  J_1(i)= 10*(states_history_1(i,1))^2+500*(1 + cos(states_history_1(i,3)+pi))^2 ...
%  + 1*(states_history_1(i,2))^2 + 15*(states_history_1(i,4))^2;
% J_2(i)= 10*(states_history_2(i,1))^2+500*(1 + cos(states_history_2(i,3)+pi))^2 ...
%  + 1*(states_history_2(i,2))^2 + 15*(states_history_2(i,4))^2;
% J_3(i)= 10*(states_history_3(i,1))^2+500*(1 + cos(states_history_3(i,3)+pi))^2 ...
%  + 1*(states_history_3(i,2))^2 + 15*(states_history_3(i,4))^2;
% J_4(i)= 10*(states_history_4(i,1))^2+500*(1 + cos(states_history_4(i,3)+pi))^2 ...
%  + 1*(states_history_4(i,2))^2 + 15*(states_history_4(i,4))^2;
% J_5(i)= 10*(states_history_5(i,1))^2+500*(1 + cos(states_history_5(i,3)+pi))^2 ...
%  + 1*(states_history_5(i,2))^2 + 15*(states_history_5(i,4))^2;
% J_6(i)= 10*(states_history_6(i,1))^2+500*(1 + cos(states_history_6(i,3)+pi))^2 ...
%  + 1*(states_history_6(i,2))^2 + 15*(states_history_6(i,4))^2;
% J_7(i)= 10*(states_history_7(i,1))^2+500*(1 + cos(states_history_7(i,3)+pi))^2 ...
%  + 1*(states_history_7(i,2))^2 + 15*(states_history_7(i,4))^2;
% J_8(i)= 10*(states_history_8(i,1))^2+500*(1 + cos(states_history_8(i,3)+pi))^2 ...
%  + 1*(states_history_8(i,2))^2 + 15*(states_history_8(i,4))^2;
% J_9(i)= 10*(states_history_9(i,1))^2+500*(1 + cos(states_history_9(i,3)+pi))^2 ...
%  + 1*(states_history_9(i,2))^2 + 15*(states_history_9(i,4))^2;
% J_10(i)= 10*(states_history_10(i,1))^2+500*(1 + cos(states_history_10(i,3)+pi))^2 ...
%  + 1*(states_history_10(i,2))^2 + 15*(states_history_10(i,4))^2;
% J_noisy(i)= 10*(states_history_noisy(i,1))^2+500*(1 + cos(states_history_noisy(i,3)+pi))^2 ...
%  + 1*(states_history_noisy(i,2))^2 + 15*(states_history_noisy(i,4))^2;
% J_1_control(i) = u_opt_1(i)*u_opt_1(i);
% J_2_control(i) = u_opt_2(i)*u_opt_2(i);
% J_3_control(i) = u_opt_3(i)*u_opt_3(i);
% J_4_control(i) = u_opt_4(i)*u_opt_4(i);
% J_5_control(i) = u_opt_5(i)*u_opt_5(i);
% J_6_control(i) = u_opt_6(i)*u_opt_6(i);
% J_7_control(i) = u_opt_7(i)*u_opt_7(i);
% J_8_control(i) = u_opt_8(i)*u_opt_8(i);
% J_9_control(i) = u_opt_9(i)*u_opt_9(i);
% J_10_control(i) = u_opt_10(i)*u_opt_10(i);
% J_noisy_control(i) = u_opt_noisy(i)*u_opt_noisy(i);
% 
% J_1_quadratic(i) = (states_history_1(i,:))*Q*(states_history_1(i,:)');
% J_2_quadratic(i) = (states_history_2(i,:))*Q*(states_history_2(i,:)');
% J_3_quadratic(i) = (states_history_3(i,:))*Q*(states_history_3(i,:)');
% J_4_quadratic(i) = (states_history_4(i,:))*Q*(states_history_4(i,:)');
% J_5_quadratic(i) = (states_history_5(i,:))*Q*(states_history_5(i,:)');
% J_6_quadratic(i) = (states_history_6(i,:))*Q*(states_history_6(i,:)');
% J_7_quadratic(i) = (states_history_7(i,:))*Q*(states_history_7(i,:)');
% J_8_quadratic(i) = (states_history_8(i,:))*Q*(states_history_8(i,:)');
% J_9_quadratic(i) = (states_history_9(i,:))*Q*(states_history_9(i,:)');
% J_10_quadratic(i) = (states_history_10(i,:))*Q*(states_history_10(i,:)');
% 
% end
% 
% for i=1:1:size(x_NMPC,1)
% J_NMPC(i) = (x_NMPC(i,:))*Q*(x_NMPC(i,:)');
% J_NMPC2(i)= 10*(x_NMPC(i,1))^2+500*(1 + cos(x_NMPC(i,3)+pi))^2 ...
%  + 1*(x_NMPC(i,2))^2 + 15*(x_NMPC(i,4))^2;
% end
% for i=1:1:size(u_NMPC,1)
% J_NMPC_control(i) = u_NMPC(i)*u_NMPC(i);
% end
% figure(7)
% t = linspace(0,50,size(states_history_1,1))/10;
% 
% for i=1:1:size(J_1,2)  
%    J_std(i) = std([J_1(i);J_2(i);J_3(i);...
%        J_4(i);J_5(i);J_6(i);...
%        J_7(i);J_8(i);J_9(i);J_10(i)]);
%    J_mean(i) = mean([J_1(i);J_2(i);J_3(i);...
%        J_4(i);J_5(i);J_6(i);...
%        J_7(i);J_8(i);J_9(i);J_10(i)]);
%    J_std_quad(i) = std([J_1_quadratic(i);J_2_quadratic(i);J_3_quadratic(i);...
%        J_4_quadratic(i);J_5_quadratic(i);J_6_quadratic(i);...
%        J_7_quadratic(i);J_8_quadratic(i);J_9_quadratic(i);J_10_quadratic(i)]);
%    J_mean_quad(i) = mean([J_1_quadratic(i);J_2_quadratic(i);J_3_quadratic(i);...
%        J_4_quadratic(i);J_5_quadratic(i);J_6_quadratic(i);...
%        J_7_quadratic(i);J_8_quadratic(i);J_9_quadratic(i);J_10_quadratic(i)]);
% end
% 
% errorbar(t, J_mean(:),J_std(:));
% hold on
% % errorbar(t, J_mean_quad(:),J_std_quad(:));
% % plot(t, J_noisy(:),'b');
% % plot(t_NMPC, J_NMPC2(:),'g');
% plot(t_NMPC, J_NMPC(:),'r');
% itm_formatfig('LatexWide')
% title(['Running Cost closed loop trajectory'],'Interpreter','latex');
% xlabel('$t$ [s]','Interpreter','latex');
% ylabel('Running Cost','Interpreter','latex');
% legend({'Mean Cost SBMPC','Cost NMPC'},'Interpreter','latex')
% 
% print('-depsc', 'running_cost_det.eps')
% print('-dpng', 'running_cost_det.png')
% 
% 
% 
% 
% figure(8)
% for i=1:1:size(J_1,2)  
%    J_std_control(i) = std([J_1_control(i);J_2_control(i);J_3_control(i);...
%        J_4_control(i);J_5_control(i);J_6_control(i);...
%        J_7_control(i);J_8_control(i);J_9_control(i);J_10_control(i)]);
%    J_mean_control(i) = mean([J_1_control(i);J_2_control(i);J_3_control(i);...
%        J_4_control(i);J_5_control(i);J_6_control(i);...
%        J_7_control(i);J_8_control(i);J_9_control(i);J_10_control(i)]);
% end
% 
% errorbar(t, J_mean_control(:),J_std_control(:));
% hold on
% % plot(t, J_noisy_control(:),'b');
% plot(t(1:50), J_NMPC_control(:),'r');
% itm_formatfig('LatexWide')
% title(['Quadratic Control Change closed loop trajectory'],'Interpreter','latex');
% xlabel('$t$ [s]','Interpreter','latex');
% ylabel('Quadratic Control Change','Interpreter','latex');
% legend({'Mean Cost of the SBMPC','Cost NMPC'},'Interpreter','latex')
% 
% print('-depsc', 'control_change_det.eps')
% print('-dpng', 'control_change_det.png')

%%

% figure (10)
% 
% t = linspace(0,5,size(states_history_1,1));
% % t_NMPC = linspace(0,5,size(x_NMPC(1:5:end,1),1));
% % plot(t_NMPC,x_NMPC(1:5:end,3),'r')
% plot(t,states_history_2(:,3))
% hold on
% plot(t,states_history_noisy(:,3))
% plot(t,states_history_alpha(:,3))
% plot(t,states_history_horizon(:,3))
% plot(t,states_history_lambda(:,3))
% plot(t,states_history_samples(:,3))
% 
% 
% % data = open('dt001nmpc.mat');
% % t = linspace(0,5,size( data.states_history,1));
% % plot(t,data.states_history(:,3))
% 
% % t = linspace(0,5,size(x_NMPC,1));
% % plot(t,x_NMPC(:,3))
% 
% t_dt = linspace(0,5,size(states_history_dt_small(1:5:end,1),1));
% plot(t_dt,states_history_dt_small(1:5:end,3),'.')
% 
% % t = linspace(0,5,size(states_history_intial_input,1));
% % plot(t,states_history_intial_input(:,3),'.')
% 
% plot([0 5],[0 0],'k')
% plot([0 5],[2*pi 2*pi],'k')
% 
% 
% % title(['Parameter Variation SBMPC'],'Interpreter','latex');
% xlabel('t [s]');
% ylabel('theta [rad]');
% legend({'SBMPC default','Sigma = 2.0, samples = 2000','alpha = 0.2' ...
%     ,'N = 15','lambda = 5', ...
%     'samples = 2000','Delta t = 0.01'})
% axis tight
% itm_formatfig('LatexWide')
% 
% print('-depsc', 'parameter_study_det_new.eps')
% print('-dpng', 'parameter_study_det_new.png')





%% dt=0.01

% figure(1)

% for i=1:1:size(states_history_dt_1,1)
%     
%    x_std(i,:) = std([states_history_dt_1(i,1);states_history_dt_2(i,1);states_history_dt_3(i,1);...
% %        states_history_dt_4(i,1);states_history_dt_5(i,1);states_history_dt_6(i,1);...
%        states_history_dt_7(i,1);states_history_dt_8(i,1);states_history_dt_9(i,1);states_history_dt_10(i,1)]);
%    y_std(i,:) = std([states_history_dt_1(i,2);states_history_dt_2(i,2);states_history_dt_3(i,2);...
% %        states_history_dt_4(i,2);states_history_dt_5(i,2);states_history_dt_6(i,2);...
%        states_history_dt_7(i,2);states_history_dt_8(i,2);states_history_dt_9(i,2);states_history_dt_10(i,2)]);
%    x_mean(i,:) = mean([states_history_dt_1(i,1);states_history_dt_2(i,1);states_history_dt_3(i,1);...
% %        states_history_dt_4(i,1);states_history_dt_5(i,1);states_history_dt_6(i,1);...
%        states_history_dt_7(i,1);states_history_dt_8(i,1);states_history_dt_9(i,1);states_history_dt_10(i,1)]);
%    y_mean(i,:) = mean([states_history_dt_1(i,2);states_history_dt_2(i,2);states_history_dt_3(i,2);...
% %        states_history_dt_4(i,2);states_history_dt_5(i,2);states_history_dt_6(i,2);...
%        states_history_dt_7(i,2);states_history_dt_8(i,1);states_history_dt_9(i,1);states_history_dt_10(i,1)]);
% 
% end

% plot(x_mean(:,1) + sin(x_mean(:,3))*0.5,0.2+ 0.05 + cos(x_mean(:,3))*0.5)

% 
% figure(2)
% plot(states_history_dt_1(:,1) + sin(states_history_dt_1(:,3))*0.5,0.2+ 0.05 + cos(states_history_dt_1(:,3))*0.5)
% hold on    
% plot(states_history_dt_2(:,1) + sin(states_history_dt_2(:,3))*0.5,0.2+ 0.05 + cos(states_history_dt_2(:,3))*0.5)
% plot(states_history_dt_3(:,1) + sin(states_history_dt_3(:,3))*0.5,0.2+ 0.05 + cos(states_history_dt_3(:,3))*0.5)
% % plot(states_history_dt_4(:,1) + sin(states_history_dt_4(:,3))*0.5,0.2+ 0.05 + cos(states_history_dt_4(:,3))*0.5)
% % plot(states_history_dt_5(:,1) + sin(states_history_dt_5(:,3))*0.5,0.2+ 0.05 + cos(states_history_dt_5(:,3))*0.5)
% % plot(states_history_dt_6(:,1) + sin(states_history_dt_6(:,3))*0.5,0.2+ 0.05 + cos(states_history_dt_6(:,3))*0.5)
% plot(states_history_dt_7(:,1) + sin(states_history_dt_7(:,3))*0.5,0.2+ 0.05 + cos(states_history_dt_7(:,3))*0.5)
% % plot(states_history_dt_8(:,1) + sin(states_history_dt_8(:,3))*0.5,0.2+ 0.05 + cos(states_history_dt_8(:,3))*0.5)
% plot(states_history_dt_9(:,1) + sin(states_history_dt_9(:,3))*0.5,0.2+ 0.05 + cos(states_history_dt_9(:,3))*0.5)
% plot(states_history_dt_10(:,1) + sin(states_history_dt_10(:,3))*0.5,0.2+ 0.05 + cos(states_history_dt_10(:,3))*0.5)
% 
% itm_formatfig('LatexWide')
% title(['Pendulum trajectories for different SBMPC simulations'],'Interpreter','latex');
% xlabel('$x$[m]','Interpreter','latex');
% ylabel('$y$[m]','Interpreter','latex');
% legend({'Simulation 1','Simulation 2','Simulation 3','Simulation 4','Simulation 5', ...
%     'Simulation 6','Simulation 7'},'Interpreter','latex')

% print('-depsc', 'Pendulum_trajectory_det.eps')
% print('-dpng', 'Pendulum_trajectory_det.png')




% t = linspace(0,50,size(states_history_dt_1,1))/10;
% t_NMPC = linspace(0,50,1+size(x_NMPC,1)/10)/10;
% % for i = 1:1:4
% f=figure(3);
% % p = uipanel('Parent',f); 
% % p.Title = 'Pendulum on a Cart States - SBMPC simulations'; 
% % p.TitlePosition = 'centertop'; 
% % p.FontSize = 12;
% % p.FontWeight = 'bold';
% 
% i=1;
% ax1 = subplot(2,2,1);
% plot(t,states_history_dt_1(:,i))
% hold on
% plot(t,states_history_dt_2(:,i))
% plot(t,states_history_dt_3(:,i))
% plot(t,states_history_dt_4(:,i))
% plot(t,states_history_dt_5(:,i))
% plot(t,states_history_dt_6(:,i))
% plot(t,states_history_dt_7(:,i))
% plot(t,states_history_dt_8(:,i))
% plot(t,states_history_dt_9(:,i))
% plot(t,states_history_dt_10(:,i))
% % plot(t,states_history_10(:,i))
% plot(t_NMPC,x_NMPC(1:10:end,i),'.r')
% t = linspace(0,5,size(states_history_intial_input,1));
% plot(t,states_history_intial_input(:,i),'-.')
% % plot(t,states_history_noisy(:,i),'*b')
% xlabel('t [s]');
% ylabel('x [m]');
% set(gca,'XTick',0:1:5)
% axis tight
% % legend({'Simulation 1','Simulation 2','Simulation 3','Simulation 4','Simulation 5', ...
% %     'Simulation 6','Simulation 7','Simulation 8','Simulation 9','Simulation 10',},'Interpreter','latex')
% 
% i=2;
% ax2 = subplot(2,2,2);
% plot(t,states_history_dt_1(:,i))
% hold on
% plot(t,states_history_dt_2(:,i))
% plot(t,states_history_dt_3(:,i))
% plot(t,states_history_dt_4(:,i))
% plot(t,states_history_dt_5(:,i))
% plot(t,states_history_dt_6(:,i))
% plot(t,states_history_dt_7(:,i))
% plot(t,states_history_dt_8(:,i))
% plot(t,states_history_dt_9(:,i))
% plot(t,states_history_dt_10(:,i))
% % plot(t,states_history_10(:,i))
% plot(t_NMPC,x_NMPC(1:10:end,i),'.r')
% t = linspace(0,5,size(states_history_intial_input,1));
% plot(t,states_history_intial_input(:,i),'-.')
% % plot(t,states_history_noisy(:,i),'*b')
% xlabel('t [s]');
% ylabel('dot{x} [m/s]');
% set(gca,'XTick',0:1:5)
% % legend({'Simulation 1','Simulation 2','Simulation 3','Simulation 4','Simulation 5', ...
% %     'Simulation 6','Simulation 7','Simulation 8','Simulation 9','Simulation 10',},'Interpreter','latex')
% axis tight
% 
% i=3;
% ax3 = subplot(2,2,3);
% plot(t,states_history_dt_1(:,i))
% hold on
% plot(t,states_history_dt_2(:,i))
% plot(t,states_history_dt_3(:,i))
% plot(t,states_history_dt_4(:,i))
% plot(t,states_history_dt_5(:,i))
% plot(t,states_history_dt_6(:,i))
% plot(t,states_history_dt_7(:,i))
% plot(t,states_history_dt_8(:,i))
% plot(t,states_history_dt_9(:,i))
% plot(t,states_history_dt_10(:,i))
% % plot(t,states_history_10(:,i))
% plot(t_NMPC,x_NMPC(1:10:end,i),'.r')
% t = linspace(0,5,size(states_history_intial_input,1));
% plot(t,states_history_intial_input(:,i),'-.')
% % plot(t,states_history_noisy(:,i),'*b')
% xlabel('t [s]');
% ylabel('theta [rad]');
% set(gca,'XTick',0:1:5)
% % legend({'Simulation 1','Simulation 2','Simulation 3','Simulation 4','Simulation 5', ...
% %     'Simulation 6','Simulation 7','Simulation 8','Simulation 9','Simulation 10',},'Interpreter','latex')
% axis tight
% 
% i=4;
% ax4 = subplot(2,2,4);
% plot(t,states_history_dt_1(:,i))
% hold on
% plot(t,states_history_dt_2(:,i))
% plot(t,states_history_dt_3(:,i))
% plot(t,states_history_dt_4(:,i))
% plot(t,states_history_dt_5(:,i))
% plot(t,states_history_dt_6(:,i))
% plot(t,states_history_dt_7(:,i))
% plot(t,states_history_dt_8(:,i))
% plot(t,states_history_dt_9(:,i))
% plot(t,states_history_dt_10(:,i))
% % plot(t,states_history_10(:,i))
% plot(t_NMPC,x_NMPC(1:10:end,i),'.r')
% t = linspace(0,5,size(states_history_intial_input,1));
% plot(t,states_history_intial_input(:,i),'-.')
% % plot(t,states_history_noisy(:,i),'*b')
% xlabel('t [s]');
% ylabel('dot{theta} [rad/s]');
% 
% set(gca,'XTick',0:1:5)
% % legend({'Simulation 1','Simulation 2','Simulation 3','Simulation 4','Simulation 5', ...
% %     'Simulation 6','Simulation 7','Simulation 8','Simulation 9','Simulation 10',},'Interpreter','latex')
% % axis tight
% 
% 
% 
% itm_formatfig('LatexWide')
% % title(['Pendulum on a Cart States - SBMPC simulations'],'Interpreter','latex');
% 
% % legend({'Sim 1','Sim 2','Sim 3','Sim 4','Sim 5', ...
% %     'Sim 6','Sim 7','Sim 8','Sim 9','Sim 10','NMPC'},'Interpreter','latex')
% print('-depsc', 'Pendulum_states_det_new.eps')
% print('-dpng', 'Pendulum_states_det_new.png')
% 
% % 
Q = diag([100,1,100,1]); % Weight matrix (states)
for i=1:1:size(states_history_dt_1,1)
 J_1(i)= 10*(states_history_dt_1(i,1))^2+500*(1 + cos(states_history_dt_1(i,3)+pi))^2 ...
 + 1*(states_history_dt_1(i,2))^2 + 15*(states_history_dt_1(i,4))^2;
J_2(i)= 10*(states_history_dt_2(i,1))^2+500*(1 + cos(states_history_dt_2(i,3)+pi))^2 ...
 + 1*(states_history_dt_2(i,2))^2 + 15*(states_history_dt_2(i,4))^2;
J_3(i)= 10*(states_history_dt_3(i,1))^2+500*(1 + cos(states_history_dt_3(i,3)+pi))^2 ...
 + 1*(states_history_dt_3(i,2))^2 + 15*(states_history_dt_3(i,4))^2;
J_4(i)= 10*(states_history_dt_4(i,1))^2+500*(1 + cos(states_history_dt_4(i,3)+pi))^2 ...
 + 1*(states_history_dt_4(i,2))^2 + 15*(states_history_dt_4(i,4))^2;
J_5(i)= 10*(states_history_dt_5(i,1))^2+500*(1 + cos(states_history_dt_5(i,3)+pi))^2 ...
 + 1*(states_history_dt_5(i,2))^2 + 15*(states_history_dt_5(i,4))^2;
J_6(i)= 10*(states_history_dt_6(i,1))^2+500*(1 + cos(states_history_dt_6(i,3)+pi))^2 ...
 + 1*(states_history_dt_6(i,2))^2 + 15*(states_history_dt_6(i,4))^2;
J_7(i)= 10*(states_history_dt_7(i,1))^2+500*(1 + cos(states_history_dt_7(i,3)+pi))^2 ...
 + 1*(states_history_dt_7(i,2))^2 + 15*(states_history_dt_7(i,4))^2;
J_8(i)= 10*(states_history_dt_8(i,1))^2+500*(1 + cos(states_history_dt_8(i,3)+pi))^2 ...
 + 1*(states_history_dt_8(i,2))^2 + 15*(states_history_dt_8(i,4))^2;
J_9(i)= 10*(states_history_dt_9(i,1))^2+500*(1 + cos(states_history_dt_9(i,3)+pi))^2 ...
 + 1*(states_history_dt_9(i,2))^2 + 15*(states_history_dt_9(i,4))^2;
J_10(i)= 10*(states_history_dt_10(i,1))^2+500*(1 + cos(states_history_dt_10(i,3)+pi))^2 ...
 + 1*(states_history_dt_10(i,2))^2 + 15*(states_history_dt_10(i,4))^2;
% J_noisy(i)= 10*(states_history_noisy(i,1))^2+500*(1 + cos(states_history_noisy(i,3)+pi))^2 ...
%  + 1*(states_history_noisy(i,2))^2 + 15*(states_history_noisy(i,4))^2;
J_1_control(i) = u_opt_dt_1(i)*u_opt_dt_1(i);
J_2_control(i) = u_opt_dt_2(i)*u_opt_dt_2(i);
J_3_control(i) = u_opt_dt_3(i)*u_opt_dt_3(i);
J_4_control(i) = u_opt_dt_4(i)*u_opt_dt_4(i);
J_5_control(i) = u_opt_dt_5(i)*u_opt_dt_5(i);
J_6_control(i) = u_opt_dt_6(i)*u_opt_dt_6(i);
J_7_control(i) = u_opt_dt_7(i)*u_opt_dt_7(i);
J_8_control(i) = u_opt_dt_8(i)*u_opt_dt_8(i);
J_9_control(i) = u_opt_dt_9(i)*u_opt_dt_9(i);
J_10_control(i) = u_opt_dt_10(i)*u_opt_dt_10(i);
% J_noisy_control(i) = u_opt_noisy(i)*u_opt_noisy(i);

J_1_quadratic(i) = (states_history_dt_1(i,:))*Q*(states_history_dt_1(i,:)');
J_2_quadratic(i) = (states_history_dt_2(i,:))*Q*(states_history_dt_2(i,:)');
J_3_quadratic(i) = (states_history_dt_3(i,:))*Q*(states_history_dt_3(i,:)');
J_4_quadratic(i) = (states_history_dt_4(i,:))*Q*(states_history_dt_4(i,:)');
J_5_quadratic(i) = (states_history_dt_5(i,:))*Q*(states_history_dt_5(i,:)');
J_6_quadratic(i) = (states_history_dt_6(i,:))*Q*(states_history_dt_6(i,:)');
J_7_quadratic(i) = (states_history_dt_7(i,:))*Q*(states_history_dt_7(i,:)');
J_8_quadratic(i) = (states_history_dt_8(i,:))*Q*(states_history_dt_8(i,:)');
J_9_quadratic(i) = (states_history_dt_9(i,:))*Q*(states_history_dt_9(i,:)');
J_10_quadratic(i) = (states_history_dt_10(i,:))*Q*(states_history_dt_10(i,:)');

end

for i=1:1:size(x_NMPC,1)
J_NMPC(i) = (x_NMPC(i,:))*Q*(x_NMPC(i,:)');
J_NMPC2(i)= 10*(x_NMPC(i,1))^2+500*(1 + cos(x_NMPC(i,3)+pi))^2 ...
 + 1*(x_NMPC(i,2))^2 + 15*(x_NMPC(i,4))^2;
end
for i=1:1:size(u_NMPC,1)
J_NMPC_control(i) = u_NMPC(i)*u_NMPC(i);
end
figure(7)
t = linspace(0,50,size(states_history_dt_1,1))/10;

for i=1:1:size(J_1,2)  
   J_std(i) = std([J_1(i);J_2(i);J_3(i);...
       J_4(i);J_5(i);J_6(i);J_8(i);...
       J_7(i);J_9(i);J_10(i)]);
   J_mean(i) = mean([J_1(i);J_2(i);J_3(i);...
       J_4(i);J_5(i);J_6(i);J_8(i);...
       J_7(i);J_9(i);J_10(i)]);
   J_std_quad(i) = std([J_1_quadratic(i);J_2_quadratic(i);J_3_quadratic(i);...
       J_4_quadratic(i);J_5_quadratic(i);J_6_quadratic(i);J_8_quadratic(i);...
       J_7_quadratic(i);J_9_quadratic(i);J_10_quadratic(i)]);
   J_mean_quad(i) = mean([J_1_quadratic(i);J_2_quadratic(i);J_3_quadratic(i);...
       J_4_quadratic(i);J_5_quadratic(i);J_6_quadratic(i);J_8_quadratic(i);...
       J_7_quadratic(i);J_9_quadratic(i);J_10_quadratic(i)]);
end

errorbar(t(1:10:end), J_mean(1:10:end)',J_std(1:10:end)');
hold on
% errorbar(t, J_mean_quad(:),J_std_quad(:));
% plot(t, J_noisy(:),'b');
% plot(t_NMPC, J_NMPC2(:),'g');
t_NMPC = linspace(0,50,size(x_NMPC,1))/10;
plot(t_NMPC, J_NMPC(:),'r');
set(gca,'XTick',0:1:5)
itm_formatfig('LatexNarrow')
% title(['Running Cost closed loop trajectory'],'Interpreter','latex');
xlabel('t [s]');
ylabel('running cost');
% legend({'Mean cost SBMPC','Cost NMPC'},'Interpreter','latex')

print('-depsc', 'running_cost_det_new_narrow.eps')
print('-dpng', 'running_cost_det_new_narrow.png')
% 
figure(8)
for i=1:1:size(J_1,2)  
   J_std_control(i) = std([J_1_control(i);J_2_control(i);J_3_control(i);...
       J_4_control(i);J_5_control(i);J_6_control(i);J_8_control(i);...
       J_7_control(i);J_9_control(i);J_10_control(i)]);
   J_mean_control(i) = mean([J_1_control(i);J_2_control(i);J_3_control(i);...
       J_4_control(i);J_5_control(i);J_6_control(i);J_8_control(i);...
       J_7_control(i);J_9_control(i);J_10_control(i)]);
end

errorbar(t(1:10:end), J_mean_control(1:10:end),J_std_control(1:10:end));
hold on
% plot(t, J_noisy_control(:),'b');
t = linspace(0,50,size(J_NMPC_control,2))/10;
plot(t, J_NMPC_control(:),'r');
set(gca,'XTick',0:1:5)
itm_formatfig('LatexNarrow')
% title(['Quadratic Control Change closed loop trajectory'],'Interpreter','latex');
xlabel('t [s]');
ylabel('quadratic control cost');
% legend({'Mean cost of the SBMPC','Cost NMPC'},'Interpreter','latex')

print('-depsc', 'control_change_det_new_narrow.eps')
print('-dpng', 'control_change_det_new_narrow.png')

figure
t = linspace(0,5,size(u_NMPC,1));
plot(t, u_NMPC(:));
hold on
t = linspace(0,5,size( u_opt_dt_1,1));
plot(t, u_opt_dt_1);
t = linspace(0,5,size( u_opt_dt_1,1));
plot(t, u_opt_intial_input);
% data = open('dt001nmpc.mat');
% t = linspace(0,5,size( data.input_history,1));
% plot(t, data.input_history);


% title(['Control over time'],'Interpreter','latex');
xlabel('$t$ [s]');
ylabel('$u$ [N]');
itm_formatfig('LatexWide')
legend({'NMPC','SBMPC','SBMPC + optimal initial input'})
% 
 print('-depsc', 'control_comparison_new_wide.eps')
print('-dpng', 'control_comparison_new_wide.png')

%error
% figure
% desired_x = [0 0 0 0];
% 
% squared_errorNMPC=(x_NMPC - desired_x).^2;
% 
% squared_errorSBMPC = (states_history_dt_1 - desired_x).^2;
% t = linspace(0,5,size(x_NMPC,1));
% plot(t,squared_errorNMPC(:,1) ,'r');
% hold on
% t = linspace(0,5,size(states_history_dt_1,1));
% plot(t,squared_errorSBMPC(:,1) ,'b');
% itm_formatfig('LatexNarrow')
% title(['Squared Error $x$ position'],'Interpreter','latex');
% xlabel('$t$[s]','Interpreter','latex');
% ylabel('error [m]','Interpreter','latex');
% legend({'NMPC','SBMPC'},'Interpreter','latex')
% print('-depsc', 'error_NMPC_SBMPC_x.eps')
% print('-dpng', 'error_NMPC_SBMPC_x.png')
% figure
% t = linspace(0,5,size(x_NMPC,1));
% plot(t,squared_errorNMPC(:,2) ,'r');
% hold on
% t = linspace(0,5,size(states_history_dt_1,1));
% plot(t,squared_errorSBMPC(:,2) ,'b');
% itm_formatfig('LatexNarrow')
% title(['Squared Error $\dot{x}$ velocity'],'Interpreter','latex');
% xlabel('$t$[s]','Interpreter','latex');
% ylabel('error [m/s]','Interpreter','latex');
% legend({'NMPC','SBMPC'},'Interpreter','latex')
% print('-depsc', 'error_NMPC_SBMPC_xdot.eps')
% print('-dpng', 'error_NMPC_SBMPC_xdot.png')
% figure
% t = linspace(0,5,size(x_NMPC,1));
% plot(t,squared_errorNMPC(:,3) ,'r');
% hold on
% t = linspace(0,5,size(states_history_dt_1,1));
% plot(t,squared_errorSBMPC(:,3) ,'b');
% itm_formatfig('LatexNarrow')
% title(['Squared Error $\theta$ angle'],'Interpreter','latex');
% xlabel('$t$[s]','Interpreter','latex');
% ylabel('error [rad]','Interpreter','latex');
% legend({'NMPC','SBMPC'},'Interpreter','latex')
% print('-depsc', 'error_NMPC_SBMPC_theta.eps')
% print('-dpng', 'error_NMPC_SBMPC_theta.png')
% figure
% t = linspace(0,5,size(x_NMPC,1));
% plot(t,squared_errorNMPC(:,4) ,'r');
% hold on
% t = linspace(0,5,size(states_history_dt_1,1));
% plot(t,squared_errorSBMPC(:,4) ,'b');
% itm_formatfig('LatexNarrow')
% title(['Squared Error $\dot{\theta}$ angular velocity'],'Interpreter','latex');
% xlabel('$t$[s]','Interpreter','latex');
% ylabel('error [rad/s]','Interpreter','latex');
% legend({'NMPC','SBMPC'},'Interpreter','latex')
% print('-depsc', 'error_NMPC_SBMPC_thetadot.eps')
% print('-dpng', 'error_NMPC_SBMPC_thetadot.png')