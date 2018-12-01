%% Nonlinear MPC for an Inverted Pendulum
% This code presents an implementation of a Nonlinear Model Predictive
% Controller for a Inverted Pendulum carried by a car. The force applied
% to the car swings the pendulum and controls its angle.
clc
clear all
close all
tic
disp('Initialize parameters');
% Initialize parameters

% System parameters
sys.m = .5;  % pendulum mass (kg)
sys.M = 1;  % car mass  (kg)
sys.g = 9.81;   % gravity force (m/s^2)
sys.l = 0.5;    % pendulum length (m)
sys.damp = 0;    % friction/damping of the car (N)
sys.dt = 0.1; % Timestep (s)

% Initial states
x_initial = 0; % initial x position
x_d_initial = 0; % initial x acceleration
phi_initial = pi;   % initial angle pendulum
phi_d_initial = 0;  % initial angle velocity
initial_conditions = [x_initial;
    x_d_initial;
    phi_initial;
    phi_d_initial];

% Desired states
x_des = 0;  % desired x position
x_d_des = 0; % desired x acceleration
phi_des = 0; % desired angle
phi_d_des = 0; % desired angle velocity
desired_states = [x_des;
    x_d_des;
    phi_des;
    phi_d_des];

% MPC variables
N = 20; % Horizon (s)
simulation_time = 10; % simulation time (s)
u_opt = zeros(N,1); % Vector with optimal input
lb = -13*ones(N,1); %Lower boundary for the input of the system (N)
ub = 13*ones(N,1);  %Upper boundary for the input of the system (N)
options = optimoptions('fmincon','Algorithm','sqp','Display','none');   % Options for the optimizer

% Visualization variables
states_history = []; % save all the states over time
input_history = []; % save all the applied optimal inputs

% Initilaize simulation
disp('Initialize simulation');

% helper variable
x_0 = initial_conditions;
tspan = [0 sys.dt];

for time = 0:sys.dt:simulation_time
    disp(['Time in the simulation: ' , num2str(time)]);
    
    % calculates the cost
    cost_function = @(u) pendulumFunction(u,x_0,desired_states,N,sys);
    % minimize the unconstrained problem
    u_opt = fmincon(cost_function,u_opt,[],[],[],[],lb,ub,[],options);
    % u_opt = fmincon(cost_function,u_opt,[],[],[],[],[],[],[],options);
    
    % applies the control input
       pendel = @(t,x)[x(2);
        (sys.l*sys.m*sin(x(3))*x(4)^2 + u_opt(l) - sys.g*sys.m*cos(x(3))*sin(x(3)))/(sys.M + sys.m*(sin(x(3))^2));
        x(4);
        (-sys.m*sys.l*x(4)^2*sin(x(3))*cos(x(3))-u_opt(l)*cos(x(3))+sys.g*sin(x(3))*(sys.m+sys.M))/((sin(x(3))^2)*sys.m*sys.l + sys.M*sys.l)];
    

        [t,x] = ode45(pendel, tspan, x_0);
        x_0 = x(end,:)';
    % Saves the states and inputs
    states_history = [states_history;
        x_0'];
    x_0'
    u_opt(1)
    input_history = [input_history;
        u_opt(1)];
   u_opt = [u_opt(2:size(u_opt,1)); 0]; 
end
disp('Simulation finished');
toc
%% Plot the important information
close all
disp('Plotting graphs');
t = linspace(0,simulation_time,simulation_time/sys.dt + 1);
figure(1)
plot(t,states_history(:,1)), title('Position vs. Time')
drawnow
figure(2)
plot(t,states_history(:,3)), title('Angle vs. Time')
drawnow
figure(3)
plot(t,input_history(:)), title('Input vs. Time')
drawnow
disp('Initialize animation');

% Small animation of the pendulum (still needs to be optimized)
figure(4)
pause(1);
car_width = 0.3;
car_height = 0.2;
ball_diameter = 0.1;
axis([min(min(states_history(:,1)),-0.55)-0.5 max(max(states_history(:,1)),0.55)+0.5 -0.55 0.55+0.4]);
axis equal
drawnow
current_axis = gca;
for i=1:1:size(states_history,1)-1
    
    % Plots the title and the time passed of the animation
    title(['Inverted Pendulum Time: ' num2str(i*sys.dt,'%.2f') 's'])
    % Draw the car
    car = rectangle('Position',[states_history(i,1)-car_width/2 0.05 car_width car_height]);
    car.EdgeColor = 'r';
    car.FaceColor = [0.7 0.1 0.1];
    % Draw the wheels
    wheel_left = rectangle('Position',[states_history(i,1)-car_width/3 0.00 0.05 0.05],'Curvature',[1 1]);
    wheel_left.EdgeColor = 'k';
    wheel_left.FaceColor = [0 0 0];
    wheel_right = rectangle('Position',[states_history(i,1)+car_width/2-car_width/3 0.00 0.05 0.05],'Curvature',[1 1]);
    wheel_right.EdgeColor = 'k';
    wheel_right.FaceColor = [0 0 0];
    % Draw the pendulum
    pendel_obj = line([states_history(i,1),states_history(i,1) + sin(states_history(i,3))*sys.l],[car_height+0.05, car_height+ 0.05 + cos(states_history(i,3))*sys.l], 'LineWidth',2);
    hold on
%     sz = linspace(0.1,20,size(states_history(:,1),1));
%     sz = ones(1,size(states_history(i,1),1))*2;
    % Draw the end of the pendulum trajectory and ball
    ball =rectangle('Position',[(states_history(i,1)+sin(states_history(i,3))*sys.l-ball_diameter/2) car_height+0.05+cos(states_history(i,3))*sys.l-ball_diameter/2 ball_diameter ball_diameter],'Curvature',[1 1]);
    ball.EdgeColor = 'r';
    ball.FaceColor = [1 0 0];
    s = scatter(states_history(i,1) + sin(states_history(i,3))*sys.l,car_height+ 0.05 + cos(states_history(i,3))*sys.l,2);    
    s.MarkerEdgeColor = 'r';
    s.MarkerFaceColor = [1.0 0.0 0.0];
    hold on
    % Draw the floor
    plot([min(min(states_history(:,1)),-0.55)-0.5 max(max(states_history(:,1)),0.55)+0.5],[0 0],'b','LineWidth',2)
    drawnow
    pause(sys.dt);
    if i<size(states_history,1)-1
        delete(pendel_obj);
        delete(car)
        delete(wheel_left)
        delete(wheel_right)
        delete(ball)
    end
end

disp('Animation finished');

