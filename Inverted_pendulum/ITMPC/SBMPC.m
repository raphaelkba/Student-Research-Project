%% Sampling-Based MPC
% This code shows an implementation of the Sampling-Based MPC
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% Inputs:
% system_dynamics: dynamics of the system of interest
% param: parameters of te MPPI Controller
%        timesteps: Horizon of the MPPI
%        samples: number of samples
%        iteration: number of iterations for the MPPI
%        xmeasure: initial conditions from the states of the system
%        x_desired: desired final states, for the cost function
%        tmeasure: intial time
%        dT: delta T
%        u: initial input sequence
%        u_UB: control upper bound
%        u_LB: control lower bound
%        lambda: inverse temperature
%        sigma: covariance of the control input
%        gamma: control cost parameter

function [tmeasure, states_history, u_opt] = SBMPC(system_dynamics, param, running_cost, terminal_cost)

% Optmization hyperparameters
timesteps = param.timesteps; %Horizon(in the paper its called timesteps)
samples = param.samples;  %Number of samples
iterations = param.iterations; % Number of iterations
xmeasure = param.xmeasure;    %Initial conditions
x_desired = param.x_desired;
tmeasure = param.tmeasure;    % Initial time
dT = param.dT;    % Timestep
u = param.u; % Initial input sequence
u_UB = param.u_UB; %Control upper bound
u_LB = param.u_LB; %Control lower bound
system_noise = param.system_noise; %Noise of the system (dW)

% Control hyperparameters
lambda = param.lambda; %Inverse temperature
gamma = param.gamma;
sigma = param.sigma; %Covariance
alpha = param.alpha;

% Initialize matrices
states_history = [];
trajectory_history = [];
u_opt = [];
dW_history = [];
S_save = [];

cost = 0;
u_opt = zeros(1,size(u,2));
save_cost = [cost];

states_history = [states_history; xmeasure];
v = zeros(timesteps-1, size(u,2));
rng('default');
rng(8);

% Begin MPC iterations
for loop = 0:1:iterations
    
    [u, save_trajectory, S] = gettrajectories(param,u...
        ,system_dynamics,running_cost,terminal_cost,loop,xmeasure,tmeasure);
    
        %     save the first optimal input
        u_opt = [u_opt; u(1,:)];
        
        %   Set the optimal input to the system        
        for state=1:1:size(xmeasure,2)
            dW(state) = sqrt(dT)*normrnd(0,system_noise(state),[1,1]);
        end
        [t_f, next_state] = ode45(@(t,x)system_dynamics(t, x ,u(1,:),dW), [tmeasure tmeasure+dT], xmeasure);
        
        
        xmeasure = next_state(end,:);
        tmeasure = t_f(end,:);
        
        % Save data        
        dW_history = [dW_history;dW];
        states_history = [states_history; xmeasure];
        
        %     Shift the input vector to the left and leave the new last input value
        %     as the same as the last one
        for t = 1:1:timesteps-1
            u(t,:) = u(t+1,:);
        end
        
        %     Initialize the new last input with a zero
        u(end,:) = 0;
        
        % Plot the current stand
        plot_trajectories(states_history,save_trajectory,tmeasure,S)
        
end

fname = 'name.mat';
save(fname,'u_opt');
save(fname,'states_history','-append');
save(fname,'trajectory_history','-append');
save(fname,'param','-append');
save(fname,'S_save','-append');

%% Plot closed loop trajectories

t = linspace(0,t_(end,:),size(states_history,1));
figure
ax1 = subplot(2,2,1);
title(['x(1) closed loop trajectory']);
xlabel('t[s]');
ylabel('x(1)[m]');
grid on;
hold on;
plot(t,states_history(:,1),'-r');
axis tight;
drawnow

ax2 = subplot(2,2,2);
title(['x(2) closed loop trajectory']);
xlabel('t[s]');
ylabel('x(2)[m/s]');
grid on;
hold on;
plot(t,states_history(:,2),'-r');
axis tight;
drawnow

ax3 = subplot(2,2,3);
title(['x(3) closed loop trajectory']);
xlabel('t[s]');
ylabel('x(3)[deg]');
grid on;
hold on;
% plot(t,rad2deg(states_history(:,3)),'-b');
plot(t,states_history(:,3),'-b');
axis tight;
drawnow

ax4 = subplot(2,2,4);
title(['x(4) closed loop trajectory']);
xlabel('t[s]');
ylabel('x(4)[deg/s]');
grid on;
hold on;
% plot(t,rad2deg(states_history(:,4)),'-b');
plot(t,states_history(:,4),'-b');
axis tight;
drawnow
% itm_formatfig('LatexWide')

figure
plot(t,rad2deg(states_history(:,3))), title('Angle vs. Time')
xlabel('t[s]');
ylabel('x(3)[deg]');
drawnow
% itm_formatfig('LatexWide')
figure
t = linspace(0,iterations,size(u_opt,2));
plot(t,u_opt), title('Input vs. Time')
xlabel('t[s]');
ylabel('u_opt[N]');
drawnow
% itm_formatfig('LatexWide')
end


% calculates the weights for each sample of the system
%  input:
%        S: sample cost
%        lambda: inverse temperature
%        samples: number of samples (can be subs to size(S,*))
% Output: importance weights
function weights = calculate_weights(S,lambda,samples)

%     Calculate weights
n = 0;
weights = zeros(1,samples);
beta = min(S);
for k=1:1:samples-1
    n = n + exp(-(1/lambda)*(S(k) - beta));
end
for k=1:1:samples-1
    weights(k) = (exp(-(1/lambda)*(S(k)-beta)))/n;
end

end


%   Plot all trajectories
%   input: states_history: all states trajectories
%          save_trajectory: current trajectory of the sample
%          tmeasure: current time
%          S: cost          
function plot_trajectories(states_history,save_trajectory,tmeasure,S)

figure(1)
cla
m = .5;  % pendulum mass (kg)
M = 1;  % car mass  (kg)
g = 9.81;   % gravity force (m/s^2)
l = 0.5;    % pendulum length (m)
car_width = 0.3;
car_height = 0.2;
ball_diameter = 0.1;

% Draw the car
car = rectangle('Position',[states_history(end,1)-car_width/2 0.05 car_width car_height]);
car.EdgeColor = 'r';
car.FaceColor = [0.7 0.1 0.1];
% Draw the wheels
wheel_left = rectangle('Position',[states_history(end,1)-car_width/3 0.00 0.05 0.05],'Curvature',[1 1]);
wheel_left.EdgeColor = 'k';
wheel_left.FaceColor = [0 0 0];
wheel_right = rectangle('Position',[states_history(end,1)+car_width/2-car_width/3 0.00 0.05 0.05],'Curvature',[1 1]);
wheel_right.EdgeColor = 'k';
wheel_right.FaceColor = [0 0 0];
% Draw the pendulum
pendel_obj = line([states_history(end,1),states_history(end,1) + sin(states_history(end,3))*l],[car_height+0.05, car_height+ 0.05 + cos(states_history(end,3))*l], 'LineWidth',2);
hold on
% Draw the end of the pendulum trajectory and ball
ball =rectangle('Position',[(states_history(end,1)+sin(states_history(end,3))*l-ball_diameter/2) car_height+0.05+cos(states_history(end,3))*l-ball_diameter/2 ball_diameter ball_diameter],'Curvature',[1 1]);
ball.EdgeColor = 'r';
ball.FaceColor = [1 0 0];
s = scatter(states_history(end,1) + sin(states_history(end,3))*l,car_height+ 0.05 + cos(states_history(end,3))*l,2);
s.MarkerEdgeColor = 'r';
s.MarkerFaceColor = [1.0 0.0 0.0];
hold on
% Draw the floor
plot([-3 3],[0 0],'b','LineWidth',2)
axis([-3 3 -0.55 0.55+0.4]);

% Normalize cost for plotting the
colormap = winter;
colorbar
A = [1:size(colormap,1)/100:size(colormap,1)]';
B = [1:1:size(colormap,1)]';

newcolormap = interp1(B, colormap ,A);
S_norm = (S - min(S))/(max(S) - min(S)+0.000001);

temperature_range = [0:1/98:1];
temperature = zeros(1, size(S_norm,2));
color = zeros(size(S_norm,2),3,1);

for i=1:1:size(S_norm,2)
    temperature(i) = find(temperature_range < round(S_norm(i),3)+0.01 & temperature_range > round(S_norm(i),3)-0.01,1);
    color(i,:,:) = newcolormap(temperature(i),:,:);
end

for k=1:1:size(save_trajectory,3)
    plot(save_trajectory(:,1,k) + sin(save_trajectory(:,3,k))*l,car_height+ 0.05 + cos(save_trajectory(:,3,k))*l,'Color', color(k,:,:))
end

% plot states
figure(2)

ax1 = subplot(4,1,1);
t = linspace(0,tmeasure,size(states_history,1));
plot(t,[states_history(:,1)],'-r','DisplayName','Current Trajectory x')
drawnow
ax2 = subplot(4,1,2);
t = linspace(0,tmeasure,size(states_history,1));
plot(t,rad2deg([states_history(:,3)]),'-r','DisplayName','Current Trajectory x dot')
%     plot(t,states_history(:,3),'-r','DisplayName','Current Trajectory')
xlabel('t[s]');
ylabel('x(3)[deg]');
drawnow
ax3 = subplot(4,1,3);
t = linspace(0,tmeasure,size(states_history,1));
plot(t,[states_history(:,2)],'-r','DisplayName','Current Trajectory phi')
drawnow
ax4 = subplot(4,1,4);
t = linspace(0,tmeasure,size(states_history,1));
plot(t,rad2deg([states_history(:,4)]),'-r','DisplayName','Current Trajectory phi dot')
drawnow

%     itm_formatfig('LatexWide')
end


%   Sample the trajectories
%   input: param: controller parameters
%          u: initial control sequence
%          system_dynamics: system dynamics
%          running_cost: running cost function
%          terminal_cost: terminal cost function
%          loop: current loop number
%          xmeasure: current states
%          tmeasure: current time
function [u, save_trajectory, S] = gettrajectories(param,u...
    ,system_dynamics,running_cost,terminal_cost,loop,xmeasure,tmeasure)


timesteps = param.timesteps; %Horizon(in the paper its called timesteps)
samples = param.samples;  %Number of samples
x_desired = param.x_desired;
dT = param.dT;    % Timestep
u_UB = param.u_UB; %Control upper bound
u_LB = param.u_LB; %Control lower bound
sigma = param.sigma; %Covariance
alpha = param.alpha;
lambda = param.lambda; %Inverse temperature
gamma = param.gamma;


% Initialize the costs to zero
S = zeros(1, samples-1);

% Sampling
for k = 1:1:samples-1
    
    save_trajectory(1,:,k) = xmeasure(end,:); % Save all the calculate traejctories
    
    % Set the initial state for every trajectory
    x_0 = xmeasure(end,:);
    t_0 = tmeasure(end,:);
    
    % Apply the stochastic input for the given horizon
    tic
    for t = 2:1:timesteps
        for i=1:1:size(u,2)
            
            noise(i,t-1,k) = normrnd(0,sigma(i),[1,1]);
            
        end
        
        % Apply control with noise
        if k < (1 - alpha)*samples
            v(t-1,:) = u(t-1,:) + noise(:,t-1,k)';
        else
            % Aplly just the noise
            v(t-1,:) = noise(:,t-1,k)';
        end
        
        % Verifies if the control respects its lower and upper bounds
        for i=1:1:size(u,2)
            if  v(t-1,i) > u_UB(i)
                v(t-1,i) = u_UB(i);
            elseif v(t-1,i) < u_LB(i)
                v(t-1,i) = u_LB(i);
            end
        end
        
        
        %  Set it to the system
        l = t-1; %Actual time step
        
        dW = zeros(1, size(x_0,2)); % No system noise for sampling

        [x_] = ode1(@(t,x)system_dynamics(t, x ,v(l,:),dW),[t_0 t_0+dT], x_0); % ODE 1 for fast sampling
        t_0 = t_0+dT; %Set new time
        x_0 = x_(end,:);
        
        % Save the calculated trajectory
        save_trajectory(t,:,k) = x_0;
        
        
        % Calculate the total cost
        [cost] = running_cost(x_0,x_desired);
        S(k) = S(k) +  cost  + gamma*u(t-1,:)*inv(diag(ones(1, size(u,2))).*sigma)*v(t-1,:)';
        
        
    end
    
    % Terminal cost
    S(k) = S(k) + terminal_cost(x_0,x_desired);
    toc
    
    % Display relevant information
    disp(['Smp: ', num2str(k), ' itr: ', num2str(loop), ' Sample Cost: ' num2str(S(k))]);
    
    
end

%     Calculate the weights
weights = calculate_weights(S,lambda,samples);


%     Push the control distibution to the optimal one
for t=2:1:timesteps-1
    
    weights_sum = zeros(1,size(u,2));
    
    for k=1:1:samples-1
        weights_sum = weights_sum + (weights(k)*noise(:,t-1,k))';
    end
    
    weights_vector(t-1,:) = weights_sum;
    
end

%  Update

% Apply Savitzky-Golay convulutional filtering for smoothness in the
% weights! and not the control input(??)
% framelen = 13;
% order = 3;
% sgf = sgolayfilt(weights_vector,order,framelen);

% Plot the old weights and smoothed ones
% figure(1)
% hold off
% y = linspace(0,size(u,1),size(u,1));
% plot(y,sgf(:,1),'b');
% hold on
% plot(y,weights_vector(:,1),'r');


for t=2:1:timesteps-1
    u(t-1,:) = u(t-1,:) + weights_vector(t-1,:);
    %     u(t-1,:) = u(t-1,:) + sgf(t-1,:);
    %     u(t-1,:) = sgolayfilt(u,order,framelen);
    % Checks if the new input sequence satifies the input constraints
    
    
    for i = 1:1:size(u,2)
        if(u(t-1,i) < u_LB(i))
            u(t-1,i) = u_LB(i);
        elseif (u(t-1,i) > u_UB(i))
            u(t-1,i) = u_UB(i);
        end
    end
end

% figure(10)
% hold off
% y = linspace(0,size(u,1),size(u,1));
% plot(y,u(:,1),'b');
% u = sgolayfilt(u,order,framelen);
%
%
% hold on
% plot(y,u(:,1),'r');


end




