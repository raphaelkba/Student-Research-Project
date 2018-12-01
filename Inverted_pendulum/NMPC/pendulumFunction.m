function J = pendulumFunction(u,x,x_desired,N,sys, inv_pe)
%% Cost calculation for every iteration of N steps horizon
%   Calculates the cost of the optimization problem for N steps horizon and
%   returns it.
%
%   Inputs:
%   u - optimization variable
%   x - current states
%       vector with 4 states: position x, acceleration x, angle theta,
%       angular acceleration theta
%   x_desired - desired states
%       vector with 4 desired states: position x, acceleration x, angle theta,
%       angular acceleration theta
%   u_opt - optimal input calculated in the last step
%   N - time horizon (tspawn*N*s)
%   sys - system parameters
%   sys.m - pendulum mass (kg)
%   sys.M - car mass (kg)
%   sys.g - gravity force (m/s^2)
%   sys.l - pendulum lenght (m)
%   sys.damp - friction/damping of the car (N)
%   sys.dt - timestep (s)
%   Outputs:
%   J - Final cost of the cost function

% Initialize parameters
R = 0.001; % Weight matrix (input)
Q = diag([100,1,100,1]); % Weight matrix (states)
J = 0; %Cost

m = sys.m;  % pendulum mass
M = sys.M;  % car mass
g = sys.g;   % gravity of earth
l = sys.l;    % pendulum length
dt = sys.dt;    %  timestep
x_0 = x; %  initial conditions
tspan = [0 dt];

% Loop through the prediction step.
for k=1:N
    
    % get the next state
    [t,x] = ode45(@(t,x) inv_pe(t, x ,u(k),sys), tspan, x_0);

    x = x(end,:)';
    % calculate the cost
    J = J + (x-x_desired)'*Q*(x-x_desired);
    
    % rate of change cost
    J = J + u(k)'*R*u(k);
    
    % Update x_0
    x_0 = x;


end

