%% Sampling-Based MPC
% Initialization of the Sampling-Based MPC problem and animation of the cart
% pendulum

clc
clear all;
close all;
warning off


% Optimization hyperparameters
param.timesteps = 15; %Horizon(in the paper its called timesteps)
param.samples = 1000;  %Number of samples
param.iterations = 500; % Number of iterations
param.xmeasure = [0 0 pi 0];    %Initial conditions (x-position,x-velocity,angle,anuglar velocity)
param.x_desired = [0 0 0 0];    %Desired states (x-position,x-velocity,angle,angular velocity)
param.tmeasure = 0;    % Initial time
param.dT = .1;    % Timestep
param.u = zeros(param.timesteps,1); % Initial input sequence
% param.u=normrnd(0,20,[param.timesteps,1]);
% data = open('u_opt_N15.mat');  %Optimal solution given by the
%                                   unconstrained nmpc with 1.5 seconds horizon
% data = open('u_opt_N20.mat'); %Optimal solution given by the
%                                 % unconstrained nmpc with 2.0 seconds horizon
% data = open('input_exact_lin.mat'); %Optimal solution given by the
%                                 % unconstrained nmpc with 2.0 seconds horizon
% data = open('dt001nmpc.mat'); %Optimal solution given by the
% %                                 % unconstrained nmpc with 2.0 seconds horizon
% param.u = data.input_history;
param.u_UB = 20; %Control upper bound
param.u_LB = -20; %Control lower bound
param.system_noise = 0*[0.1 0.1 0.1 0.1]; % Noise of the system (dW)


% Control hyperparameters
param.lambda = 1; %Inverse temperature
param.sigma = 0.5; %Covariance
param.alpha = 0.01; %Base distribution parameters (0 < alpha < 1)
param.gamma = param.lambda*(1-param.alpha);

tic
inital_time_cpu = cputime;

[t_all, x_all, u_all] = SBMPC(@system_dynamics, param, @running_cost,@terminal_cost);

param.time_normal = toc
param.cpu_time = cputime - inital_time_cpu

% Inverted pendulum system dynamics
% pendel(1): x velocity
% pendel(2): x acceleration
% pendel(3): phi angular velocity
% pendel(4): phi angular acceleration
function pendel = system_dynamics(t, x ,u, dW)
% System parameters
m = .5;  % pendulum mass (kg)
M = 1;  % car mass  (kg)
g = 9.81;   % gravity force (m/s^2)
l = 0.5;    % pendulum length (m)

                pendel = [x(2) + dW(1);
        (l*m*sin(x(3))*x(4)^2 + u(1)  - g*m*cos(x(3))*sin(x(3)))/(M + m*(sin(x(3))^2)) + dW(2);
        x(4)+ dW(3);
        (-m*l*x(4)^2*sin(x(3))*cos(x(3))-u(1)*cos(x(3))+(g*sin(x(3)))*(m+M))/((sin(x(3))^2)*m*l + M*l)+ dW(4)]; 


end

function [J] = running_cost(x, x_desired)

% calculate the cost
 J_(1) = 10*(x(1))^2 ;
 J_(2) = 500*(1 + cos(x(3)+pi))^2;
 J_(3) = 1*(x(2))^2;
 J_(4) = 15*(x(4))^2;
% Q = diag([100,1,100,1]);
% J=(x-x_desired)'*Q*(x-x_desired);
 J = sum(J_);

end

function [J] = terminal_cost(x, x_desired)

% J_(1) = 0*(x(1))^2;
% J_(2) = 5000*(1 + cos(x(3)+pi))^2;
% J_(3) = 0*(x(2))^2;
% J_(4) = 100*(x(4))^2;
% J = sum(J_);
J = 0;

end
