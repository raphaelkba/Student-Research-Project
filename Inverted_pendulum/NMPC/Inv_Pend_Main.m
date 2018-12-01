% Initialization of the Nonlinear MPC problem and animation of the cart
% pendulum
clc
clear all;
close all;


% Parameters for the MPC
mpciterations = 100;
N             = 20;
T             = 0.1;
options = optimoptions('fmincon','Algorithm','sqp','MaxFunctionEvaluations',100000000,'MaxIterations',100000000,'StepTolerance',1e-8);%,'Display','none');   % Options for the optimizer
% Initial measurements
tmeasure      = 0.0;
xmeasure      = [0.0, 0.0, pi, 0.0];
desired_states = [0, 0, 0, 0]';
u0            = 0*ones(1,N);
% System parameters
sys.m = .5;  % pendulum mass (kg)
sys.M = 1;  % car mass  (kg)
sys.g = 9.81;   % gravity force (m/s^2)
sys.l = 0.5;    % pendulum length (m)
sys.dt = T; % Timestep (s)

% Initialize optimization
[t, x, u] = Nonlinear_MPC(@inverted_pendulum,@constraints, ...
    @terminalconstraints, @linearconstraints, ...
    mpciterations, N, T, tmeasure, xmeasure,...
    u0,desired_states, options, sys, 5, @printHeader, @printClosedloopData, @plotTrajectories);


% Plots
states_history = x;
input_history = u;
% % close all
% disp('Plotting graphs');
% simulation_time = mpciterations*T;
% % t = linspace(0,simulation_time,simulation_time/sys.dt);
% figure(1)
% plot(t,states_history(:,1)), title('Position vs. Time')
% drawnow
% figure(2)
% plot(t,states_history(:,3)), title('Angle vs. Time')
% drawnow
% % figure(3)
% % plot(t,input_history(:)), title('Input vs. Time')
% drawnow
disp('Initialize animation');
%%
inverted_pendulum_animation(states_history, sys, mpciterations, T);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Definition of the NMPC functions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Function to define the constraints of t,x and u.
% c is a vector of inequality constraints
% cew is a vector of equality constraints
function [c,ceq] = constraints(t, x, u)
c   = [];
ceq = [];
end

% Function to define the terminal constraints of t,x.
% c is a vector of inequality constraints
% cew is a vector of equality constraints
function [c,ceq] = terminalconstraints(t, x)
% ceq(1) = x(1);
% ceq(2) = x(2);
% ceq(3) = x(3);
% ceq(4) = x(4);
c = [];
ceq = [];

end

% Linear constraints derivated from the linear system
% A: A matrix for the inequality constraints
% b: b matrix for the inequality constraints
% Aeq: A matrix for the equality constraints
% beq: b matrix for the equality constraints
% lb: lower bound for the control input
% ub: upper bound for the control input
function [A, b, Aeq, beq, lb, ub] = linearconstraints(t, x, u)
A   = [];
b   = [];
Aeq = [];
beq = [];
lb  = -10;
ub  = 10;
end

% Inverted pendulum system dynamics
% pendel(1): x velocity
% pendel(2): x acceleration
% pendel(3): phi angular velocity
% pendel(4): phi angular acceleration
function pendel = inverted_pendulum(t, x ,u ,sys)

                pendel = [x(2);
        (sys.l*sys.m*sin(x(3))*x(4)^2 + u(1) - sys.g*sys.m*cos(x(3))*sin(x(3)))/(sys.M + sys.m*(sin(x(3))^2));
        x(4);
        (-sys.m*sys.l*x(4)^2*sin(x(3))*cos(x(3))-u(1)*cos(x(3))+sys.g*sin(x(3))*(sys.m+sys.M))/((sin(x(3))^2)*sys.m*sys.l + sys.M*sys.l)];
end




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Definition of output format
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function printHeader()
fprintf('   k  |      u(k)        x(1)        x(2)        x(3)        x(4)     Time\n');
fprintf('--------------------------------------------------------------------------\n');
end

function printClosedloopData(mpciter, u, x, t_Elapsed)

fprintf(' %3d  | %+11.6f %+11.6f %+11.6f %+11.6f  %+6.3f', mpciter, ...
    u(1,1), x(1), x(2), x(3), x(4), t_Elapsed);

end


function plotTrajectories(T, t0, x0, u, x_open, t_open ,lb, ub , mpciter)
sys.m = .5;  % pendulum mass (kg)
sys.M = 1;  % car mass  (kg)
sys.g = 9.81;   % gravity force (m/s^2)
sys.l = 0.5;    % pendulum length (m)
sys.dt = T; % Timestep (s)

[t_intermediate, x_intermediate] = ode45(@(t,x) inverted_pendulum(t, x ,u(1), sys), [t0 t0+T], x0);

x = x_intermediate(size(x_intermediate,1),:);

% Plot position of the cart and angle of the pendulum closed loop
% trajectory

%Plot optimal input closed loop trajectory
figure(2);
current_axis = gca;
title(['u optimal closed loop']);
xlabel('t[s]');
ylabel('u optimal [N]');
grid on;
hold on;
bar(t_intermediate(1)+T/2,u(1),T,'FaceColor',[1 .1 .1]);
drawnow
axis([0 mpciter*T+T lb(1)-2 ub(1)+2]);
axis square;
% itm_formatfig(2,'LineWidth',2,'FontName','Arial');

% Plot position of the cart and angle of the pendulum open loop
% trajectory
figure(3)
ax1 = subplot(2,2,1);
title(['x(1) open loop trajectory']);
xlabel('t[s]');
ylabel('x(1) [m]');
grid on;
hold on;
scatter(t_open(1),x_open(1,1),'r');
drawnow
plot(t_open,x_open(:,1),'-r');
drawnow
axis tight;

ax2 = subplot(2,2,2);
title(['x(2) open loop trajectory']);
xlabel('t[s]');
ylabel('x(2) [m/s]');
grid on;
hold on;
scatter(t_open(1),x_open(1,2),'r');
drawnow
plot(t_open,x_open(:,2),'-r');
drawnow
axis tight;

ax3 = subplot(2,2,3);
title(['x(3) open loop trajectory']);
xlabel('t[s]');
ylabel('x(3) [rad]');
grid on;
hold on;
scatter(t_open(1),x_open(1,3),'b');
drawnow
plot(t_open,x_open(:,3),'-b');
drawnow
axis tight;
drawnow

ax4 = subplot(2,2,4);
title(['x(4) open loop trajectory']);
xlabel('t[s]');
ylabel('x(4) [rad/s]');
grid on;
hold on;
scatter(t_open(1),x_open(1,4),'b');
drawnow
plot(t_open,x_open(:,4),'-b');
drawnow
axis tight;
itm_formatfig('LatexWide')



figure(1);
current_axis = gca;
title(['x(1) and x(3) closed loop trajectory']);
xlabel('t[s]');
ylabel('x(1) [m], x(3) [rad]');

grid on;
hold on;
plot(t_intermediate,x_intermediate(:,1),'-r');
drawnow
plot(t_intermediate,x_intermediate(:,3),'-b');
legend('x(1)','x(3)')
drawnow
itm_formatfig('LatexWide')
% axis([min(t_intermediate) max(t_intermediate) min(min(x_intermediate(:,1)),min(x_intermediate(:,3))) max(max(x_intermediate(:,1)),max(x_intermediate(:,3)))]);
axis tight;

end

function inverted_pendulum_animation(states_history, sys, mpciter,T)
% Small animation of the pendulum (still needs to be optimized)
figure(10)
pause(1);
car_width = 0.3;
car_height = 0.2;
ball_diameter = 0.1;
axis([min(min(states_history(:,1)),-0.55)-0.5 max(max(states_history(:,1)),0.55)+0.5 -0.55 0.55+0.4]);
axis equal
drawnow
xlabel('x-axis[m]');
ylabel('y-axis[m]');
current_axis = gca;

maxTime = mpciter*T;
actualTime = 0;
step = maxTime/(size(states_history,1)-1);

for i=1:5:size(states_history,1)-1
    actualTime = actualTime + 5*step;
    % Plots the title and the time passed of the animation
    title(['Inverted Pendulum Time: ' num2str(actualTime,'%.2f') 's'])
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
%     pause(step);
    if i<size(states_history,1)-6
        delete(pendel_obj);
        delete(car)
        delete(wheel_left)
        delete(wheel_right)
        delete(ball)
    end
itm_formatfig(2)
end

disp('Animation finished');

% figure(1)
% print('-depsc', 'n_20x1x3_const.eps')
% print('-dpng', 'n_20x1x3_const.png')
% 
% figure(2)
% print('-depsc', 'n_20u_opt_const.eps')
% print('-dpng', 'n_20u_opt_const.png')
% 
% figure(3)
% print('-depsc', 'n_20subplot_const.eps')
% print('-dpng', 'n_20suplot_const.png')
% 
% figure(10)
% print('-depsc', 'n_20final_pend_const.eps')
% print('-dpng', 'n_20final_pend_const.png')

end





