%% Sampling-Based MPC
% Sampling-Based MPC implementation for the control of an inverted pendulum
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
xmeasure2 = param.xmeasure2;    %Initial conditions
x_desired2 = param.x_desired2;
xmeasure3 = param.xmeasure3;    %Initial conditions
x_desired3 = param.x_desired3;
tmeasure = param.tmeasure;    % Initial time
dT = param.dT;    % Timestep
u_1 = param.u; % Initial input sequence
u_2 = param.u2; % Initial input sequence
u_3 = param.u3; % Initial input sequence
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
states_history2 = [];
states_history3 = [];

u_opt = zeros(1,size(u_1,2));
u_opt2 = zeros(1,size(u_1,2));
u_opt3 = zeros(1,size(u_1,2));

trajectory_history = [];
S_save = [];
trajectory_history2 = [];
S_save2 = [];
trajectory_history3 = [];
S_save3 = [];
dW_history = [];
states_history = [states_history; xmeasure];
states_history2 = [states_history2; xmeasure2];
states_history3 = [states_history3; xmeasure3];
% Set random seed
% rng('default');
% rng(9);


% Begin MPC iterations
for loop = 0:1:iterations
    
    minS = 99999999999999999;
    
    %get white noise
    for state=1:1:size(xmeasure,2)
        dW(state) = sqrt(dT)*normrnd(0,system_noise(state),[1,1]);
    end
    xmeasure_ =xmeasure;
    xmeasure2_ =xmeasure2;
    xmeasure3_ =xmeasure3;
    for i=1:1:size(u_1,1)
        
        % get next state for every robot
        [t_f, xmeasure_] = ode45(@(t,x)system_dynamics(t, x ,u_1(i,:),dW), [tmeasure tmeasure+dT], xmeasure_);
        [t_f, xmeasure2_] = ode45(@(t,x)system_dynamics(t, x ,u_2(i,:),dW), [tmeasure tmeasure+dT], xmeasure2_);
        [t_f, xmeasure3_] = ode45(@(t,x)system_dynamics(t, x ,u_3(i,:),dW), [tmeasure tmeasure+dT], xmeasure3_);
        xmeasure_ = xmeasure_(end,:);
        xmeasure2_ = xmeasure2_(end,:);
        xmeasure3_ = xmeasure3_(end,:);
        x_pred1(i,:) = xmeasure_(end,:);
        x_pred2(i,:) = xmeasure2_(end,:);
        x_pred3(i,:) = xmeasure3_(end,:);
        
        
        
    end
    
    % Initialize the costs to zero
    S1 = zeros(1, samples-1);
    S2 = zeros(1, samples-1);
    S3 = zeros(1, samples-1);
    
    % Initialize forward sampling
    [u_1, save_trajectory, S1] = gettrajectories(samples,xmeasure,tmeasure,sigma,timesteps,alpha,u_UB,u_LB,u_1...
        ,system_dynamics,gamma,system_noise,dT,x_desired,running_cost,S1,terminal_cost,loop...
        ,lambda,1,x_pred2,x_pred3);
    
    
    
    [u_2, save_trajectory2, S2] = gettrajectories(samples,xmeasure2,tmeasure,sigma,timesteps,alpha,u_UB,u_LB,u_2...
        ,system_dynamics,gamma,system_noise,dT,x_desired2,running_cost,S2,terminal_cost,loop...
        ,lambda,2,x_pred1,x_pred3);
    
    
    
    [u_3, save_trajectory3, S3] = gettrajectories(samples,xmeasure3,tmeasure,sigma,timesteps,alpha,u_UB,u_LB,u_3...
        ,system_dynamics,gamma,system_noise,dT,x_desired3,running_cost,S3,terminal_cost,loop...
        ,lambda,3,x_pred1,x_pred2);
    
    
    %     save the first optimal input
    u_opt = [u_opt; u_1(1,:)];
    u_opt2 = [u_opt2; u_2(1,:)];
    u_opt3 = [u_opt3; u_3(1,:)];
    
    % generate noise
    for state=1:1:size(xmeasure,2)
        dW(state) = sqrt(dT)*normrnd(0,system_noise(state),[1,1]);
    end
    %   Set the optimal input to the system
    [t_f, next_state] = ode45(@(t,x)system_dynamics(t, x ,u_1(1,:),dW), [tmeasure tmeasure+dT], xmeasure);
    [t_f, next_state2] = ode45(@(t,x)system_dynamics(t, x ,u_2(1,:),dW), [tmeasure tmeasure+dT], xmeasure2);
    [t_f, next_state3] = ode45(@(t,x)system_dynamics(t, x ,u_3(1,:),dW), [tmeasure tmeasure+dT], xmeasure3);
    
    xmeasure = next_state(end,:);
    xmeasure2 = next_state2(end,:);
    xmeasure3 = next_state3(end,:);
    tmeasure = t_f(end,:);
    
    % Save the data
    dW_history = [dW_history;
        dW];
    states_history = [states_history; xmeasure];
    states_history2 = [states_history2; xmeasure2];
    states_history3 = [states_history3; xmeasure3];
    trajectory_history(:,:,:,loop+1) = save_trajectory;
    S_save = [S_save;S1];
    trajectory_history2(:,:,:,loop+1) = save_trajectory2;
    S_save2 = [S_save2;S2];
    trajectory_history3(:,:,:,loop+1) = save_trajectory3;
    S_save3 = [S_save3;S3];
    
    
    %     Shift the input vector to the left and leave the new last input value
    %     as the same as the last one
    for t = 1:1:timesteps-1
        u_1(t,:) = u_1(t+1,:);
    end
    for t = 1:1:timesteps-1
        u_2(t,:) = u_2(t+1,:);
    end
    for t = 1:1:timesteps-1
        u_3(t,:) = u_3(t+1,:);
    end
    
    
    %     Initialize the new last input with a zero
    u_1(timesteps,:) = 0;
    u_2(timesteps,:) = 0;
    u_3(timesteps,:) = 0;
    
    plot_trajectories(states_history,save_trajectory,states_history2,...
        save_trajectory2,states_history3,save_trajectory3,(loop),S1,S2,S3);
    
    
end

% fname = sprintf('name.mat');
%
% save(fname,'u_opt');
% save(fname,'u_opt2','-append');
% save(fname,'u_opt3','-append');
% save(fname,'states_history','-append');
% save(fname,'states_history2','-append');
% save(fname,'states_history3','-append');
% save(fname,'trajectory_history','-append');
% save(fname,'trajectory_history2','-append');
% save(fname,'trajectory_history3','-append');
% save(fname,'S_save','-append');
% save(fname,'S_save2','-append');
% save(fname,'S_save3','-append');



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

%   Plot all trajectories
%   input: states_history: all states robot 1
%          save_trajectory: sampled trajectories robot 1
%          states_history2: all states robot 2
%          save_trajectory2: sampled trajectories robot 2
%          states_history3: all states robot 3
%          save_trajectory3: sampled trajectories robot 3
%          t: current time
%          S: cost of trajectories robot 1
%          S2:cost of trajectories robot 2
%          S3:cost of trajectories robot 3
    function plot_trajectories(states_history,save_trajectory,...
            states_history2,save_trajectory2,states_history3,save_trajectory3,t,S,S2,S3)
        
        figure(7)
        cla
        
        % Set color to each trajectory based on the cost robot 1
        colormap(gca,hot())
        colormap_ = flipud(autumn());
        A = [1:size(colormap_,1)/100:size(colormap_,1)]';
        B = [1:1:size(colormap_,1)]';
        newcolormap = interp1(B, colormap_ ,A);
        S_norm = (S - min(S))/(max(S) - min(S));
        temperature_range = [0:1/98:1];
        temperature = zeros(1, size(S_norm,2));
        color = zeros(size(S_norm,2),3,1);
        for i=1:1:size(S_norm,2)
            temperature(i) = find(temperature_range < round(S_norm(i),2)+0.01 & temperature_range > round(S_norm(i),2)-0.01,1);
            color(i,:,:) = newcolormap(temperature(i),:,:);
        end
        
        
        % Plot each robot states
        hold on
        plot(states_history(:,1),states_history(:,2),'-b','DisplayName','Current Trajectory 1')
        plot(states_history2(:,1),states_history2(:,2),'-g','DisplayName','Current Trajectory 2')
        plot(states_history3(:,1),states_history3(:,2),'-r','DisplayName','Current Trajectory 3')
        
        % Plot robot 1 trajectories
        for k=1:1:size(save_trajectory,3)
            plot(save_trajectory(:,1,k),save_trajectory(:,2,k),'Color', color(k,:,:))
        end
        
        % Set color to each trajectory based on the cost robot 2
        A = [1:size(colormap_,1)/100:size(colormap_,1)]';
        B = [1:1:size(colormap_,1)]';
        newcolormap = interp1(B, colormap_ ,A);
        S_norm = (S2 - min(S2))/(max(S2) - min(S2));
        temperature_range = [0:1/98:1];
        temperature = zeros(1, size(S_norm,2));
        color = zeros(size(S_norm,2),3,1);
        for i=1:1:size(S_norm,2)
            temperature(i) = find(temperature_range < round(S_norm(i),2)+0.01 & temperature_range > round(S_norm(i),2)-0.01,1);
            color(i,:,:) = newcolormap(temperature(i),:,:);
        end
        hold on
        % Plot robot 2 trajectories
        for k=1:1:size(save_trajectory2,3)
            plot(save_trajectory2(:,1,k),save_trajectory2(:,2,k),'Color', color(k,:,:))
        end
        
        % Set color to each trajectory based on the cost robot 3
        A = [1:size(colormap_,1)/100:size(colormap_,1)]';
        B = [1:1:size(colormap_,1)]';
        newcolormap = interp1(B, colormap_ ,A);
        S_norm = (S3 - min(S3))/(max(S3) - min(S3));
        temperature_range = [0:1/98:1];
        temperature = zeros(1, size(S_norm,2));
        color = zeros(size(S_norm,2),3,1);
        for i=1:1:size(S_norm,2)
            temperature(i) = find(temperature_range < round(S_norm(i),2)+0.01 & temperature_range > round(S_norm(i),2)-0.01,1);
            color(i,:,:) = newcolormap(temperature(i),:,:);
            
        end
        % Plot robot 3 trajectories
        for k=1:1:size(save_trajectory3,3)
            plot(save_trajectory3(:,1,k),save_trajectory3(:,2,k),'Color', color(k,:,:))
        end
        
        % Draw robots
        x_ = states_history(end,1); y_ = states_history(end,2);
        rad =0.1;
        angs = 0:pi/10:2*pi;
        y = x_ + rad*sin(angs);
        x = y_ + rad*cos(angs);
        plot(y,x,'b','LineWidth',2);
        
        x = [x_,x_ + rad*sin(-states_history(end,3) + pi/2)];
        y = [y_,y_ + rad*cos(-states_history(end,3) + pi/2)];
        %plot the line
        plot(x,y,'r','LineWidth',2);
        
        x_2 = states_history2(end,1); y_2 = states_history2(end,2);
        rad =0.1;
        angs = 0:pi/10:2*pi;
        y = x_2 + rad*sin(angs);
        x = y_2 + rad*cos(angs);
        plot(y,x,'b','LineWidth',2);
        
        x = [x_2,x_2 + rad*sin(-states_history2(end,3) + pi/2)];
        y = [y_2,y_2 + rad*cos(-states_history2(end,3) + pi/2)];
        %plot the line
        plot(x,y,'r','LineWidth',2);
        
        x_3 = states_history3(end,1); y_3 = states_history3(end,2);
        rad =0.1;
        angs = 0:pi/10:2*pi;
        y = x_3 + rad*sin(angs);
        x = y_3 + rad*cos(angs);
        plot(y,x,'b','LineWidth',2);
        
        x = [x_3,x_3 + rad*sin(-states_history3(end,3) + pi/2)];
        y = [y_3,y_3 + rad*cos(-states_history3(end,3) + pi/2)];
        %plot the line
        plot(x,y,'r','LineWidth',2);
        
        obstx = 3; obsty = 0;
        rad =2.0 - rad - 0.1;
        angs = 0:pi/10:2*pi;
        y = obstx + rad*sin(angs);
        x = obsty + rad*cos(angs);
        % plot(y,x,'b','LineWidth',2);
        
        % Draw center of the formation
        c_x = (1/3)*(states_history(end,1) + states_history2(end,1) + states_history3(end,1));
        c_y = (1/3)*(states_history(end,2) + states_history2(end,2) + states_history3(end,2));
        rad =0.05;
        angs = 0:pi/10:2*pi;
        y = c_x + rad*sin(angs);
        x = c_y + rad*cos(angs);
        plot(y,x,'k','LineWidth',2);
        
        % x = [x_,x_ + rad*sin(-states_history(end,3) +pi/2+ pi/4)];
        % y = [y_,y_ + rad*cos(-states_history(end,3) +pi/2+ pi/4)];
        % %plot the line
        % plot(x,y,'r','LineWidth',2);
        % figure
        % t=1
        % Plot desired trajectory
        t_ = [0:0.01:(t+1)/50];
        a = 2*sin(t_);
        plot(t_,a,'.r')
        
        grid on
        colormap(gca,flipud(hot()))
        h = colorbar(gca);
        % set( h, 'YDir', 'reverse' );
        axis equal
        drawnow
        
    end


%   Forward sampling of trajectories
%   input: samples: number of samples
%          xmeasure: current states
%          tmeasure: current time
%          sigma: covariance matrix
%          timesteps: number of timesteps
%          alpha: control cost variable
%          t: current time
%          u_UB: upper bound of the control
%          u_LB: lower bound of the control
%          u: current control sequence
%          system_dynamics: function with the system dynamics
%          gamma: control cost variable
%          system_noise: white noise of the system states
%          dT: time discretization
%          x_desired: desired states
%          running_cost: function with the running cost
%          S: cost vector
%          terminal_cost: function with the terminal cost
%          loop: number of the current iteration
%          lambda: inverse temperature
%          r1: number of the current robot 
%          x_pred2: prediction trajectory of neighbour robot
%          x_pred3: prediction trajectory of second neighbour robot
    function [u, save_trajectory,S] = gettrajectories(samples,xmeasure,tmeasure,sigma,timesteps,alpha,u_UB,u_LB,u...
            ,system_dynamics,gamma,system_noise,dT,x_desired,running_cost,S,terminal_cost,loop,...
            lambda,r1,x_pred2,x_pred3)
        
        save_trajectory = zeros(timesteps,size(xmeasure,2), samples -1);
        
        %Initiliaze forward sampling
        for k = 1:1:samples-1
            
            save_trajectory(1,:,k) = xmeasure(end,:);
            % Set the initial state for every trajectory
            x_0 = xmeasure(end,:);
            t_0 = tmeasure(end,:);
            
            % Get a normal distributed noise for the input control
            for i=1:1:size(u,2)
                noise(i,:,k) = normrnd(0,sigma(i),[1,timesteps]);
            end
            
            % Set penalty to false
            penalty =0;
            
            for t = 2:1:timesteps
                if k < (1 - alpha)*samples  
                    %             Get the control with noise
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
                for state=1:1:size(xmeasure,2)
                    dW(state) = 0*normrnd(0,system_noise(state),[1,1]);
                end
                
                %    [t_, x_] = ode45(@(t,x)system_dynamics(t, x ,v(l,:),dW), [t_0 t_0+dT], x_0);
                [ x_] = ode1(@(t,x)system_dynamics(t, x ,v(l,:),dW), [t_0 t_0+dT], x_0);
                t_=  t_0+dT;
                t_0 = t_(end,:);
                
                % Check for collision with other robots
                if penalty == 0
                    if (x_0(1) - x_pred2(t-1,1))^2 + (x_0(2) - x_pred2(t-1,2))^2 < 0.12^2
                        penalty = 10000000000000;
                    else
                        x_0 = x_(end,:);
                    end
                end
                if penalty == 0
                    if (x_0(1) - x_pred3(t-1,1))^2 + (x_0(2) - x_pred3(t-1,2))^2 < 0.12^2
                        
                        penalty = 10000000000000;
                    else
                        x_0 = x_(end,:);
                        
                    end
                end
                
                save_trajectory(t,:,k) = [x_0];
                
                
                
                % Calculate the cost
                [cost] = running_cost(x_0,x_desired,x_pred2(t-1,:),x_pred3(t-1,:),(loop+t-1),r1);
                S(k) =S(k) + penalty +cost + gamma*u(t-1,:)*inv(diag(ones(1, size(u,2))).*sigma)*v(t-1,:)';
                
                
                
            end
            % No terminal cost here
            S(k) = S(k);
            
            disp(['Smp: ', num2str(k), ' itr: ', num2str(loop), ' Sample Cost: ' num2str(S(k))]);
            
        end
        
        %     Calculate the weights
        weights = calculate_weights(S,lambda,samples);
        
        
        %     Push the control distibution to the optimal one
        for t=1:1:timesteps
            
            weights_sum = zeros(1,size(u,2));
            
            for k=1:1:samples-1
                weights_sum = weights_sum + (weights(k)*noise(:,t,k))';
            end
            
            weights_vector(t,:) = weights_sum;
        end
        
        %         Update
        %         order = 5;
        %         framelen = 7;
        %         sgf = sgolayfilt(weights_vector,order,framelen);
        
        for t=1:1:timesteps
            u(t,:) = u(t,:) + weights_vector(t,:);
            %         u(t,:) = u(t,:) + sgf(t,:);
            
            for i = 1:1:size(u,2)
                if(u(t,i) < u_LB(i))
                    u(t,i) = u_LB(i);
                elseif (u(t,i) > u_UB(i))
                    u(t,i) = u_UB(i);
                end
            end
        end
        
    end
end