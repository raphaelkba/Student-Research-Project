%% Sampling-Based MPC
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
function [tmeasure, states_history, u_opt] = SBMPC_SingleTrack(system_dynamics, param, running_cost, terminal_cost)


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
trajectory_history = [];
S_save = [];
dW_history = [];
states_history = [states_history; xmeasure];
save_trajectory = zeros(timesteps,size(xmeasure,2), samples -1);
% seed for the random values
% rng('default');
% rng(1);



% Create the race track
t_4_r=[-30*ones(150,1) (linspace(250,400,150))'];
t_4_l=[-35*ones(150,1) (linspace(250,400,150))'];
t_5_r=[(linspace(-30,20,200))' sqrt(625-((linspace(-30,20,200))'+5).^2)+400];
t_5_l=[(linspace(-35,25,200))' sqrt(900-((linspace(-35,25,200))'+5).^2)+400];
t_6_r=[20*ones(100,1) (linspace(400,250,100))'];
t_6_l=[25*ones(100,1) (linspace(400,250,100))'];
t_7_r=[-(linspace(-30,20,200))'-10 -sqrt(625-((linspace(-30,20,200))'+5).^2)+250];
t_7_l=[-(linspace(-35,25,200))'-10 -sqrt(900-((linspace(-35,25,200))'+5).^2)+250];
a = flip(t_7_r);
b = flip(t_7_l);
m1 = (250-400)/(-32.5-22.5);
b1 = 400/(m1*22.5);
m2 = (250-400)/(22.5+32.5);
b2 = 400/(m2*-32.5);
x_1 = linspace(22.5,-32.5);
line_1 = m1*(x_1 + 32.5) + 250;
x_2 = linspace(22.5,-32.5);
line_2 = m2*(x_2 - 22.5) + 250;
t_r=[t_5_l; flip(t_7_r );t_5_l(1,:)];
t_l=[t_5_r  ;flip(t_7_l);t_5_r(1,:);];
c1_r=[t_5_l];
c1_l=[t_5_r];
c2_r=[flip(t_7_r )];
c2_l=[flip(t_7_l)];
t_r_x=t_r(1:1:end,1);
t_r_y=t_r(1:1:end,2);
t_l_x=t_l(1:1:end,1);
t_l_y=t_l(1:1:end,2);

% get the center of the track
middlec1x =  (c1_r(1:1:end,1) + c1_l(1:1:end,1))/2;
middlec2x =  (c2_r(1:1:end,1) + c2_l(1:1:end,1))/2;
middlec1y =  (c1_r(1:1:end,2) + c1_l(1:1:end,2))/2;
middlec2y =  (c2_r(1:1:end,2) + c2_l(1:1:end,2))/2;
middle_x_ = [middlec1x middlec1y;x_1' line_1';middlec2x middlec2y ;x_2' line_2' ];
middle_x = middle_x_(1:1:end,1);
middle_y = middle_x_(1:1:end,2);


% vector with the desired path
path = [middle_x middle_y];
trajectory_history = [];

controller = robotics.PurePursuit;
controller.Waypoints = path;
controller.LookaheadDistance = 0.1;

% Begin MPC iterations
for loop = 0:1:iterations
    
    minS = 99999999999999999;
    % Initialize the costs to zero
    S = zeros(1, samples-1);
    
    % Initialize forward sampling
    [u, x_0, t_, save_trajectory,S] = gettrajectories(samples,xmeasure,tmeasure,sigma,timesteps,alpha,u_UB,u_LB,u...
        ,system_dynamics,gamma,system_noise,dT,x_desired,running_cost,S,terminal_cost,loop,...
        lambda,middle_x,middle_y);
    


        %     save the first optimal input
        u(1,5) = round(u(1,5));
        u_opt = [u_opt; u(1,:)];
        
        %   Set the optimal input to the system
        
        for state=1:1:size(xmeasure,2)
            dW(state) = sqrt(dT)*normrnd(0,system_noise(state),[1,1]);
        end
        
        [t_f, next_state] = ode45(@(t,x)system_dynamics(t, x ,u(1,:),dW), [tmeasure tmeasure+dT], xmeasure);
        %         [ next_state] = ode1(@(t,x)system_dynamics(t, x ,u(1,:),dW), [tmeasure tmeasure+dT], xmeasure);
        %         t_f = tmeasure+dT;
        x_open = xmeasure;
        t_open = tmeasure;
        next_state = real(next_state);
        %     Add noise to the output
        for state=1:1:size(next_state,2)
            xmeasure(:,state) = real(next_state(end,state));%  +  normrnd(0,system_noise(state),[1,1]);
        end
        tmeasure = t_f(end,:);
        
        % Save the data
        dW_history = [dW_history;
            dW];
        states_history = [states_history; xmeasure];
        trajectory_history(:,:,:,loop+1) = save_trajectory;
        S_save = [S_save;S];
        %     figure(2)
        %     hold off
        
        %     Shift the input vector to the left and leave the new last input value
        %     as the same as the last one
        for t = 1:1:timesteps-1
            u(t,:) = u(t+1,:);
        end
        
        %     Initialize the new last input with a zero (gear with 1)
        u(timesteps,:)= [0 0 0 0 1];
        %                 u(timesteps,:)= [0 15000/2 1/2 1/2];

    plot_trajectories(states_history,save_trajectory,path,u,S,timesteps,dT,system_dynamics);
    
    
end


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

%   Plot all trajectories
%   input: states_history: all states trajectories
%          save_trajectory: current trajectory of the sample
%          path: desired path
%          control: control sequence
%          S: trajectory costs
%          timesteps:horizon
%          dT: time discretization
%          system_dynamics: function with the system dynamics
function plot_trajectories(states_history,save_trajectory, ...
                            path,control,S,timesteps,dT,system_dynamics)


figure(7)
hold off

% create track
t_4_r=[-30*ones(150,1) (linspace(250,400,150))'];
t_4_l=[-35*ones(150,1) (linspace(250,400,150))']; 
t_5_r=[(linspace(-30,20,200))' sqrt(625-((linspace(-30,20,200))'+5).^2)+400]; 
t_5_l=[(linspace(-35,25,200))' sqrt(900-((linspace(-35,25,200))'+5).^2)+400]; 
t_6_r=[20*ones(100,1) (linspace(400,250,100))']; 
t_6_l=[25*ones(100,1) (linspace(400,250,100))']; 
t_7_r=[-(linspace(-30,20,200))'-10 -sqrt(625-((linspace(-30,20,200))'+5).^2)+250]; 
t_7_l=[-(linspace(-35,25,200))'-10 -sqrt(900-((linspace(-35,25,200))'+5).^2)+250]; 
t_r=[t_5_l; flip(t_7_r );t_5_l(1,:)];
t_l=[t_5_r  ;flip(t_7_l);t_5_r(1,:);];

axis equal % equal axis scaling
axis([-50 70 -50 450]) % plot height and width
plot(t_r(:,1),t_r(:,2)) % plot right racetrack boundary
hold on
plot(t_l(:,1),t_l(:,2)) % plot left racetrack boundary
text(1,0,'\leftarrow finish/start','HorizontalAlignment','left') % finish/start annotation
xlabel('x') % label x axis
ylabel('y') % label y axies
box % make a box around the plot


plot(states_history(:,1),states_history(:,2),'-b','DisplayName','Current Trajectory')

colormap = winter;
colorbar
A = [1:size(colormap,1)/100:size(colormap,1)]';
B = [1:1:size(colormap,1)]';

newcolormap = interp1(B, colormap ,A);
S_norm = (S - min(S))/(max(S) - min(S));
[Svalue, minidx] = min(S_norm);
temperature_range = [0:1/98:1];
temperature = zeros(1, size(S_norm,2));
color = zeros(size(S_norm,2),3,1);

if ~isinf(min(S))
% get color for each trajectory
for i=1:1:size(S_norm,2)
    if isnan(S_norm(i))
        S_norm(i)= max(S_norm);
    end
    temperature(i) = find(temperature_range < round(S_norm(i),2)+0.01 & temperature_range > round(S_norm(i),2)-0.01,1);
    color(i,:,:) = newcolormap(temperature(i),:,:);
    
end

% plot trajectories with respective color
for k=1:1:size(save_trajectory,3)
    plot(save_trajectory(:,1,k),save_trajectory(:,2,k),'Color', color(k,:,:))
end
% plot lowest cost trajectory
plot(save_trajectory(:,1,minidx),save_trajectory(:,2,minidx),'.k')
end
% Plot the current states given by the current input sequence
save_current_states = [];
x_open = states_history(end,:);
dW = zeros(1, size(states_history,2));
for j = 1:1:timesteps
    
    [x_test_] = ode1(@(t,x)system_dynamics(t, x ,control(j,:),dW), [0 dT], x_open);
    save_current_states = [save_current_states; x_test_];
    x_open = x_test_(end,:);
    
end
plot(save_current_states(:,1),save_current_states(:,2),'.r')


% plot the single-track model
l_f=1.19016;
l_r=1.37484;
l=l_f+l_r;
R=0.302;

x_ = states_history(end,1); y_ = states_history(end,2);
car = plot([x_-l_r  x_+l_f  x_+l_f  x_-l_r  x_-l_r], [y_-0.6  y_-0.6  y_+0.6  y_+0.6  y_-0.6],'LineWidth',2);
rotate(car, [0 0 1], rad2deg(states_history(end,5)),[x_ y_ 0]);
wheel_1 = plot([x_+l_f-R  x_+l_f+R  x_+l_f+R  x_+l_f-R  x_+l_f-R], [y_+1.10/2  y_+1.10/2  y_+1.10/2-0.3  y_+1.10/2-0.3  y_+1.10/2],'LineWidth',2,'Color','r');
wheel_2 = plot([x_-l_r-R  x_-l_r+R  x_-l_r+R  x_-l_r-R  x_-l_r-R], [y_+1.10/2  y_+1.10/2  y_+1.10/2-0.3  y_+1.10/2-0.3  y_+1.10/2],'LineWidth',2,'Color','b');
wheel_3 = plot([x_+l_f-R  x_+l_f+R  x_+l_f+R  x_+l_f-R  x_+l_f-R], [y_-1.10/2  y_-1.10/2  y_-1.10/2+0.3  y_-1.10/2+0.3  y_-1.10/2],'LineWidth',2,'Color','r');
wheel_4 = plot([x_-l_r-R  x_-l_r+R  x_-l_r+R  x_-l_r-R  x_-l_r-R], [y_-1.10/2  y_-1.10/2  y_-1.10/2+0.3  y_-1.10/2+0.3  y_-1.10/2],'LineWidth',2,'Color','b');
rotate(wheel_1, [0 0 1], rad2deg(states_history(end,5)),[x_ y_ 0]);
rotate(wheel_2, [0 0 1], rad2deg(states_history(end,5)),[x_ y_ 0]);
rotate(wheel_1, [0 0 1], rad2deg(control(1,1)),[x_+l_f*cos(states_history(end,5)) y_+l_f*sin(states_history(end,5)) 0]);
rotate(wheel_3, [0 0 1], rad2deg(states_history(end,5)),[x_ y_ 0]);
rotate(wheel_4, [0 0 1], rad2deg(states_history(end,5)),[x_ y_ 0]);
rotate(wheel_3, [0 0 1], rad2deg(control(1,1)),[x_+l_f*cos(states_history(end,5)) y_+l_f*sin(states_history(end,5)) 0]);

% plot desired path
plot(path(:,1), path(:,2))

% axis([min(min(save_trajectory(:,1,:)))-10 max(max(save_trajectory(:,1,:)))+10 min(min(save_trajectory(:,2,:)))-10 max(max(save_trajectory(:,2,:)))+10])
axis equal
text(x_ +2 ,y_ + 10,num2str(states_history(end,3)));
% camroll(-90);
drawnow

end



function [u, x_0, t_, save_trajectory,S] = gettrajectories(samples,xmeasure,tmeasure,sigma,timesteps,alpha,u_UB,u_LB,u...
    ,system_dynamics,gamma,system_noise,dT,x_desired,running_cost,S,terminal_cost,loop,...
    lambda,middle_x,middle_y)


for k = 1:1:samples-1
%     controller.Waypoints = path;
    save_trajectory(1,:,k) = xmeasure(end,:);
    
    % Set the initial state for every trajectory
    x_0 = xmeasure(end,:);
    t_0 = tmeasure(end,:);

    % Get a normal distributed noise for the input control
    for i=1:1:size(u,2)
          noise(i,:,k) = normrnd(0,sigma(i),[1,timesteps]);
%         noise(i,:,k) = rand(1,timesteps);
    end
    % round the input value to an integer (gear number)
    noise(5,:,k) = round(noise(5,:,k));
    
    % Apply the input for the given horizon
    for t = 2:1:timesteps
        % Save trajectory
            
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
            dW(state) = normrnd(0,system_noise(state),[1,1])*0;
        end
        
        %             [t_, x_] = ode45(@(t,x)system_dynamics(t, x ,v(l,:),dW), [t_0 t_0+dT], x_0);
        [x_] = ode1(@(t,x)system_dynamics(t, x ,v(l,:),dW), [t_0 t_0+dT], x_0);
        t_ = t_0+dT;
        x_ = real(x_);
        t_0 = t_;
                
        % search for closer point in the path
        kk = dsearchn([middle_x middle_y],[x_0(end,1) x_0(end,2)]);
        % correct the index
        if kk-1 < 1
            kk = 2;
        end
        if kk+1 > 600
            kk = 600-1;
        end
        
        % calculates the distance to the path
        x1=middle_x(kk+1);
        x2=middle_x(kk-1);
        y1=middle_y(kk+1);
        y2=middle_y(kk-1);
        distance_to_line = abs((y2-y1)*x_0(end,1) -...
            (x2-x1)*x_0(end,2) + x2*y1...
            -y2*x1)...
            /(sqrt((y2-y1)^2 +(x2-x1)^2));
        
        
        
        x_0 = x_(end,:);
        save_trajectory(t,:,k) = [x_0];
        
        % create a penalty for beeing out of the track
        penalty = 0;
        if distance_to_line > 5 % form the middle to the border of the track 5meters
            penalty = 100000000000;
        end
        
        % Calculate the cost
        [cost] = running_cost(x_0,x_desired,t-1);
        S(k) = S(k) +penalty+ cost + distance_to_line^2  + 1*gamma*u(t-1,:)*inv(diag(ones(1, size(u,2))).*sigma)*v(t-1,:)';
        
    end
    %       Terminal cost
    S(k) = S(k) + terminal_cost(x_0,x_desired);
    %recover cost
    if isnan(S(k))
        if k==1
        S(k)= 100000000000000000;   
        else
        S(k)= max(S(1:k-1));
        end
    end
    
    disp(['Smp: ', num2str(k), ' itr: ', num2str(loop), ' Sample Cost: ' num2str(S(k)), ' Velocity: ' num2str(xmeasure(3))]);
    
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
framelen = 11;
order =5;
sgf = sgolayfilt(weights_vector,order,framelen);

for t=1:1:timesteps
    %         u(t,:) = u(t,:) + weights_vector(t,:);
    u(t,:) = u(t,:) + sgf(t,:);
    
    for i = 1:1:size(u,2)
        if(u(t,i) < u_LB(i))
            u(t,i) = u_LB(i);
        elseif (u(t,i) > u_UB(i))
            u(t,i) = u_UB(i);
        end
    end
end
% plot_trajectories(states_history,save_trajectory,map,path,cost_map,u,S,timesteps,dT,system_dynamics);
end

