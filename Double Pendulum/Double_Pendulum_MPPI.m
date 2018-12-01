%% Model Predictive Path Integral Control
% Initialization of the Model Predictive Path Integral Control MPC problem and animation of the cart
% pendulum

clc
clear all;
close all;
warning off


% Optimization hyperparameters
param.timesteps = 20; %Horizon(in the paper its called timesteps)
param.samples = 20;  %Number of samples
param.iterations = 100; % Number of iterations
param.xmeasure = [0 pi pi 0 0 0];    %Initial conditions (x-position,x-velocity,angle,anuglar velocity)
param.x_desired = [0 0 0 0 0 0];    %Desired states (x-position,x-velocity,angle,angular velocity)
param.tmeasure = 0;    % Initial time
param.dT = .1;    % Timestep
param.u = zeros(param.timesteps,1); % Initial input sequence

param.u_UB = 20; %Control upper bound
param.u_LB = -20; %Control lower bound
param.system_noise = [0.01 0.01 0.01 0.01 0.01 0.01]; % Noise of the system (dW)


% Control hyperparameters
param.lambda = 1; %Inverse temperature
param.sigma = 0.9; %Covariance
param.alpha = 0.01; %Base distribution parameters (0 < alpha < 1)
param.gamma = param.lambda*(1-param.alpha);


[t_all, x_all, u_all] = MPPIControl(@system_dynamics, param, @running_cost,@terminal_cost);



% Inverted pendulum system dynamics (http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.78.6105&rep=rep1&type=pdf)
% pendel(1): x velocity
% pendel(2): phi angular velocity pendulum 1
% pendel(3): phi angular velocity pendulum 2
% pendel(4): x acceleration
% pendel(5): phi angular acceleration pendulum 1
% pendel(6): phi angular acceleration pendulum 2
function system = system_dynamics(t, r ,u, dW)

% System parameters
    m0 = 1.5;   %mass cart
    m1 = 0.5;   %mass link 1
    m2 = 0.75;  %mass link 2
    L1 = 0.5;   %lenght link 1
    L2 = 0.75;  %lenght link 2
    g = 9.81;   %gravity constant
    
    x = r(1);   % cart position
    theta_1 = r(2); %angle first joint
    theta_2 = r(3); %angle second joint
    x_dot = r(4);   %cart velocity
    theta_1_dot = r(5); % angular velocity first joint 
    theta_2_dot = r(6); %angular velocity second joint


    d1 = m0 + m1 + m2;
    d2 = (0.5*m1 + m2)*L1;
    d3 = 0.5*m2*L2;
    d4 = ((1/3)*m1 + m2)*L2^2;
    d5 = (1/2)*m2*L1*L2;
    d6 = (1/3)*m2*L2^2;
    f1 = ((1/2)*m1 + m2)*L1*g;
    f2 = (1/2)*m2*L2*g;

    D = [d1 d2*cos(theta_1) d3*cos(theta_2);
                    d2*cos(theta_1) d4 d5*cos(theta_1-theta_2);
                   d3*cos(theta_2) d5*cos(theta_1-theta_2) d6];
    
    C = [0 -d2*sin(theta_1)*theta_1_dot -d3*sin(theta_2)*theta_2_dot;
        0 0 d5*sin(theta_1-theta_2)*theta_2_dot;
        0 -d5*sin(theta_1-theta_2)*theta_1_dot 0];
    
    G = [0;
         -f1*sin(theta_1);
         -f2*sin(theta_2)];
    H = [1;0;0];
    
    system = [zeros(3,3) eye(3);zeros(3,3) -inv(D)*C]*[x; theta_1;theta_2;x_dot; theta_1_dot;theta_2_dot]+...
            [zeros(3,1); -inv(D)*G] + [zeros(3,1);inv(D)*H]*u + dW';
    

end

function [J] = running_cost(x, x_desired)

% calculate the cost
 J_(1) = 10*(x(1))^2 ;
 J_(2) = 500*(1 + cos(x(2)+pi))^2;
 J_(3) = 500*(1 + cos(x(3)+pi))^2;
 J_(4) = 10*(x(4))^2;
 J_(5) = 15*(x(5))^2;
 J_(6) = 15*(x(6))^2;
 J = sum(J_);
 

end

function [J] = terminal_cost(x, x_desired)

J_(1) = 0*(x(1))^2;
J_(2) = 5000*(1 + cos(x(3)+pi))^2;
J_(3) = 0*(x(2))^2;
J_(4) = 100*(x(4))^2;
J = sum(J_);
J = 0;

end
