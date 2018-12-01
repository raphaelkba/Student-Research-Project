%% Sampling-Based Model Predictive Control
% This code shows an implementation of the Sampling-Based MPC
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% Inputs:
% system_dynamics: dynamics of the system of interest
% param: parameters of te SBMPC Controller
%        timesteps: Horizon of the SBMPC
%        samples: number of samples
%        iteration: number of iterations for the SBMPC
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
% running_cost: function with the running cost
% terminal_cost: function with the terminal cost
% Outputs: tmeasure: total time
%          states_history: all states during the simulation
%          u_opt: vector with the used controls
function [tmeasure, states_history, u_opt] = MPPIControl(system_dynamics, param, running_cost, terminal_cost)


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
sigma = param.sigma; %Covariance
gamma = param.gamma;
alpha = param.alpha;

% Initialize matrices
states_history = [];
cost = 0;
u_opt = zeros(1,size(u,2));
save_cost = [cost];
error_history = [];
dW_history = [];
states_history = [states_history; xmeasure];
v = zeros(timesteps-1, size(u,2));
% Set random seed
% rng('default');
% rng(1);


% Begin MPC iterations
for loop = 0:1:iterations

    % Initialize the costs to zero
    S = zeros(samples-1,iterations+1);
    % Get a normal distributed noise for the input control
    noise = normrnd(0,sigma,[size(u,2),timesteps,samples-1]);

    % Initialize forward sampling
    for k = 1:1:samples-1
        save_trajectory = zeros(timesteps,size(xmeasure,2),samples-1,iterations);
      
        % Set the initial state for every trajectory
        x_0 = xmeasure(end,:);
        t_0 = tmeasure(end,:);
        
        % Apply the input for the given horizon
        for t = 2:1:timesteps
            % Save trajectory
            
            save_trajectory(t,:,k,loop+1) = x_0;
            
            if k < (1 - alpha)*samples   
            v(t-1,:) = u(t-1,:) + noise(:,t-1,k)';            
            else
            v(t-1,:) = noise(:,t-1,k)';     
            end

            for i=1:1:size(u,2)
            if  v(t-1,i) > u_UB(i)
                v(t-1,i) = u_UB(i); 
            elseif v(t-1,i) < u_LB(i)
                v(t-1,i) = u_LB(i); 
            end
            end
            
            %  Set it to the system
            l = t-1;
                      
            dW = 0*normrnd(0,system_noise,[1,size(xmeasure,2)]);          
            
%             [t_, x_] = ode45(@(t,x)system_dynamics(t, x ,v(l,:),dW), [t_0 t_0+dT], x_0);
            [t_, x_] = ode113(@(t,x)system_dynamics(t, x ,v(l,:),dW), [t_0 t_0+dT], x_0);
            t_ = t_0+dT;
            x_0 = x_(end,:);
            t_0 = t_(end,:);
            
            % Calculate the cost
            [cost] = running_cost(x_0,x_desired);
            S(k,loop+1) = S(k,loop+1) + cost + gamma*u(t-1,:)*inv(diag(ones(1, size(u,2)))*sigma)*v(t-1,:)';

         
        end
        
        % Save data
        save_trajectory(t,:,k,loop+1) = x_0;
        % Terminal cost
        S(k,loop+1) = S(k,loop+1) + terminal_cost(x_0,x_desired);
        
        

        % Display relevant information
        disp(['Sample: ', num2str(k), ' iteration: ', num2str(loop)...
            , ' x: ' num2str(xmeasure(1)), ' m',...
            ' x_dot: ' num2str(xmeasure(2)),...
         ' phi: ' num2str(xmeasure(3)), ' degrees', ...  % ' phi: ' num2str(rad2deg(xmeasure(3))), ' degrees', ...           
           ' phi_dot: ' num2str(xmeasure(4)),...
            ' Last Input: ', num2str(u_opt(end,:)), ' Sample Cost: ' num2str(S(k))])
        
    end
%     plot_trajectories(states_history,save_trajectory,tmeasure,t_0)
%     Calculate the weights
    weights = calculate_weights(S(:,loop+1),lambda,samples);

    
    %     Push the control distibution to the optimal one
    weights_vector = zeros(timesteps,1);
    for t=1:1:timesteps
  
        weights_sum = zeros(1,size(u,2));
  
        for k=1:1:samples-1
            weights_sum = weights_sum + (weights(k)*noise(:,t,k))';
        end
        
        weights_vector(t,:) = weights_sum;
    end
    
%         Update control 
%         Filter
%         order = 5;
%         framelen = 7; 
%         sgf = sgolayfilt(weights_vector,order,framelen);   
                    
        for t=1:1:timesteps
        u(t,:) = u(t,:) + weights_vector(t,:); %No filter
%         u(t,:) = u(t,:) + sgf(t,:); % With filter

        for i = 1:1:size(u,2)
        if(u(t,i) < u_LB(i))
            u(t,i) = u_LB(i);
        elseif (u(t,i) > u_UB(i))
            u(t,i) = u_UB(i);
        end
        end
        end
        
      
    %     save the first optimal input
    u_opt = [u_opt; u(1,:)];
    
   
%   Set the optimal input to the system  
    for state=1:1:size(xmeasure,2) 
            dW(state) = sqrt(dT)*normrnd(0,system_noise(state),[1,1]);
     end
    [t_f, next_state] = ode45(@(t,x)system_dynamics(t, x ,u(1,:),dW), [tmeasure tmeasure+dT], xmeasure);
    x_open = xmeasure;
    t_open = tmeasure;
    
%     Add noise to the output
    for state=1:1:size(next_state,2) 
    xmeasure(:,state) = next_state(end,state);%  +  normrnd(0,system_noise(state),[1,1]);
    end
    tmeasure = t_f(end,:);
 
    [cost] = running_cost(x_0,x_desired);
    [cost] = terminal_cost(x_0,x_desired);
    
 
   
    % Save data
     dW_history = [dW_history;
                    dW];
    states_history = [states_history; xmeasure];            

    
    %     Shift the input vector to the left and leave the new last input value
    %     as the same as the last one
    for t = 1:1:timesteps-1
        u(t,:) = u(t+1,:);
    end
    
    %     Initialize the new last input with a zero
        u(timesteps,:) = 0;
        
end


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

%   Plot all trajectories (not good)
%   input: states_history: all states trajectories
%          save_trajectory: current trajectory of the sample
%          tmeasure: current time
%          t_0: initial time
function plot_trajectories(states_history,save_trajectory,tmeasure,t_0)
    
    figure(1)
    x0=10;
    y0=10;
    width=650;
    height=500;
    set(gcf,'units','points','position',[x0,y0,width,height]);
    ax1 = subplot(3,1,1);
    t = linspace(tmeasure,t_0,size(save_trajectory,1));    
    plot(t,save_trajectory(:,1)), title('Position vs. Time')
    xlabel('t[s]');
    ylabel('x(1)[m]');
    hold on
    t = linspace(0,tmeasure,size(states_history,1));
    plot(t,[states_history(:,1)],'-r','DisplayName','Current Trajectory')
    drawnow
    ax2 = subplot(3,1,2);
    t = linspace(tmeasure,t_0,size(save_trajectory,1));
%     plot(t,rad2deg(save_trajectory(:,3))), title('Angle vs. Time')
    plot(t,save_trajectory(:,3)), title('Angle vs. Time')
    hold on
    t = linspace(0,tmeasure,size(states_history,1));
%     plot(t,rad2deg([states_history(:,3)]),'-r','DisplayName','Current Trajectory')
    plot(t,states_history(:,3),'-r','DisplayName','Current Trajectory')
    xlabel('t[s]');
    ylabel('x(3)[deg]');
    drawnow
        ax3 = subplot(3,1,3);
%     t = linspace(tmeasure,t_0,size(save_trajectory,1));
%     plot(t,rad2deg(save_trajectory(:,3))), title('Angle vs. Time')
%     plot(save_trajectory(:,1),save_trajectory(:,3)), title('Angle vs. Time')
    hold on
    t = linspace(tmeasure,t_0,size(save_trajectory,1));
%     plot(t,rad2deg([states_history(:,3)]),'-r','DisplayName','Current Trajectory')
%     plot(states_history(:,1),states_history(:,3),'-r','DisplayName','Current Trajectory')
    plot(save_trajectory(:,1),save_trajectory(:,3)), title('Angle vs. Time')
    xlabel('t[s]');
    ylabel('x(3)[deg]');
    drawnow
    %     itm_formatfig('LatexWide')        
end




