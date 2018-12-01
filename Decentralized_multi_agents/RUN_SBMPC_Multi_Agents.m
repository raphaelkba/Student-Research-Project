%%Sampling-Based MPC
% Initialization of the Sampling-Based MPC problem

clc
clear all;
close all;
warning off



% Optmization hyperparameters
param.timesteps = 20; %Horizon(in the paper its called timesteps)
param.samples = 10;  %Number of samples
param.iterations = 600; % Number of iterations
param.xmeasure2 = [-1 0 pi/2];    %Initial conditions (x-position,x-velocity,angle,anuglar velocity)
param.x_desired = [0 0 pi/2];    %Desired states (x-position,x-velocity,angle,anuglar velocity)
param.xmeasure3 = [-2 0 pi/2];    %Initial conditions (x-position,x-velocity,angle,anuglar velocity)
param.x_desired2 = [0 0 pi/2];    %Desired states (x-position,x-velocity,angle,anuglar velocity)
param.xmeasure = [-3 0 pi/2];    %Initial conditions (x-position,x-velocity,angle,anuglar velocity)
param.x_desired3 = [0 0 pi/2];    %Desired states (x-position,x-velocity,angle,anuglar velocity)
param.tmeasure = 0;    % Initial time
param.dT = .1;    % Timestep
n_inputs = 2;

param.u =0*ones(param.timesteps,n_inputs); % Initial input sequence
param.u2 = 0*ones(param.timesteps,n_inputs); % Initial input sequence
param.u3 = 0*ones(param.timesteps,n_inputs); % Initial input sequence

param.u_UB = [10 10]; %Control upper bound
param.u_LB = [-10 -10]; %Control lower bound
param.system_noise = [0.01 0.01 0.01];


% Control hyperparameters
param.lambda = 1; %Inverse temperature
param.sigma = [1 1]; %Covariance
param.alpha = 0.01; %Base distribution parameters (0 < alpha < 1)
param.gamma = param.lambda*(1-param.alpha);

tic
inital_time_cpu = cputime;
[t_all, x_all, u_all] = MPPIControl(@system_dynamics, param, @running_cost,@terminal_cost);
param.time_normal = toc
param.cpu_time = cputime - inital_time_cpu


% Diff Driver Kinematics
% diffdrive(1): x velocity
% diffdrive(2): y velocity
% diffdrive(3): phi angular velocity

function diffdrive = system_dynamics(t, x ,u, dW)
% System parameters
        % wheel radius
       r = 0.1;
       % lenght between wheels
       L =0.4;
       %kinematics
       diffdrive = [
           (r/2)*(u(1,1)+u(1,2))*cos(x(3))+ dW(1);
           (r/2)*(u(1,1)+u(1,2))*sin(x(3))+ dW(2);  
           (r/L)*(u(1,2)-u(1,1))+ dW(3)];
            
end

function [J] = running_cost(x, x_desired, x2,x3,t,robot)

% calculate the cost
% Q = diag([10,10,0]);
% J =(x-x_desired)*Q*(x-x_desired)';
VLx = t/50;
VLy = 2*sin((t)/50);
% y_= linspace(-3,6,400);
% x_ = zeros(1,400);
% VLx = x_desired(1);
% VLy = x_desired(2);
% VLx = 6.5;
% VLy = 0;

r = 1.0;
angle = 120;
    J = 500*((x(1) - VLx + r*sin(deg2rad(angle*robot)))^2 ...
    +(x(2) - VLy + r*cos(deg2rad(angle*robot)))^2) ;
%     J = 500*((x(1)  + r*sin(deg2rad(angle*robot)))^2 ...
%     +(x(2)  + r*cos(deg2rad(angle*robot)))^2) ;
%         J = 100*( sqrt((x2(1) - x(1))^2 + (x2(2) - x(2))^2) - 3 )^2 + ...
%     100*( sqrt((x2(1) - x3(1))^2 + (x2(2) - x3(2))^2) - 3 )^2 +...
%     100*( sqrt((x(1) - x3(1))^2 +  (x(2) - x3(2))^2) - 3 )^2;
c_x = (1/3)*(x(1) + x2(1) + x3(1));
c_y = (1/3)*(x(2) + x2(2) + x3(2));

J = J+ 500*((c_x - VLx)^2 + (c_y - VLy)^2) ;

% J = J + 100*((x(1) - x2(1))^2 + (x(2) - x2(2))^2 - (x2(1) - x3(1))^2 + (x2(2) - x3(2))^2); 
% J = J + 1*((x(1)-0)^2 + (x(2)-4)^2);

end

function [J] = terminal_cost(x, x_desired)

Q = diag([0,0,0]);
J =(x-x_desired)*Q*(x-x_desired)';

end

