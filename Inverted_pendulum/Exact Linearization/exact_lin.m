%% Exact linearization 
% Simulation of an inverted pendulum with use of the exact linearization
% feedback controller for non linear sysyems


clc
clear all;
close all;
warning off


iterations = 400;
dT = 0.01;
tmeasure = 0;
xmeasure = [0 0 pi 0];
u = 0;
u_UB = 20; %Control upper bound
u_LB = -20; %Control lower bound
states_history = [];
input_history = [];
system_noise = [0.05 0.05 7 7]*0; % Noise of the system (dW)
simulation_idx = 13;
fname = sprintf('simulations/simulation_%d.mat', simulation_idx);
data = open(fname);
%  u = data.u_opt;
% data = open('u_opt_N15.mat');  %Optimal solution given by the
%                                   unconstrained nmpc with 1.5 seconds horizon
data = open('u_opt_N20.mat'); %Optimal solution given by the
u = data.input_history;
% 2.3412e+03

for i=1:1:iterations
    
    
   
    
    m = .5;  % pendulum mass (kg)
    M = 1;  % car mass  (kg)
    g = 9.81;   % gravity force (m/s^2)
    l = 0.5;    % pendulum length (m)
    fx = 0;
    pole = 5;
    pole_2 = 10;      

    u = -m*l*(xmeasure(4)^2)*sin(xmeasure(3))*cos(xmeasure(3))+(g*sin(xmeasure(3))*(m+M))...
        - fx*xmeasure(2)*cos(xmeasure(3)) + pole*xmeasure(4)*(sin(xmeasure(3))^2*m*l+M*l)...
        + pole_2*xmeasure(3)*(sin(xmeasure(3))^2*m*l+M*l);
     
%     u = -m*l*(xmeasure(4)^2)*sin(xmeasure(3))*cos(xmeasure(3))+(g*sin(xmeasure(3))*(m+M))...
%         - fx*xmeasure(2)*cos(xmeasure(3)) + pole*xmeasure(4)*(sin(xmeasure(3))^2*m*l+M*l)...
%         + pole_2*(1 + cos(x(3)+pi))*(sin(xmeasure(3))^2*m*l+M*l);
    
    states_history = [states_history; xmeasure];
    
    
    if(u < u_LB)
            u = u_LB;
        elseif (u > u_UB)
            u = u_UB;
       end 
input_history = [input_history; u];  
% u = u + normrnd(0,8.9,[1,1]);
    for state=1:1:size(xmeasure,2) 
    dW(state) = 0*sqrt(dT)*normrnd(0,system_noise(state),[1,1]);
    end
[t_f, next_state] = ode45(@(t,x)system_dynamics(t, x ,u,dW), [tmeasure tmeasure+dT], xmeasure);


tmeasure = t_f(end,:);
xmeasure = next_state(end,:);    
%     Add noise to the output
    for state=1:1:size(next_state,2) 
    
%     xmeasure(:,state) = next_state(end,state) + dW;
    end
u
xmeasure
 plot_trajectories(states_history,input_history,tmeasure)
end

input_history
save('input_exact_lin','input_history');

function plot_trajectories(states_history,input_history,tmeasure)
    
    figure(1)
    x0=10;
    y0=10;
    width=650;
    height=500;
    set(gcf,'units','points','position',[x0,y0,width,height]);
    ax1 = subplot(5,1,1);
    t = linspace(0,tmeasure,size(states_history,1));
    plot(t,[states_history(:,1)],'-b')
    xlabel('t[s]');
    ylabel('x [m]');
    drawnow
    ax2 = subplot(5,1,2);
    t = linspace(0,tmeasure,size(states_history,1));
    plot(t,states_history(:,2),'-b')
    xlabel('t[s]');
    ylabel('x dot[m/s]');
    drawnow
     ax3 = subplot(5,1,3);
    t = linspace(0,tmeasure,size(states_history,1));
    plot(t,rad2deg([states_history(:,3)]),'-r')
%     plot(t,states_history(:,3),'-r','DisplayName','Current Trajectory')
    xlabel('t[s]');
    ylabel('theta [deg]');
    drawnow
     ax4 = subplot(5,1,4);
    t = linspace(0,tmeasure,size(states_history,1));
    plot(t,rad2deg([states_history(:,4)]),'-r')
%     plot(t,states_history(:,4),'-r','DisplayName','Current Trajectory')
    xlabel('t[s]');
    ylabel('theta dot [deg/s]');
    drawnow
        ax5 = subplot(5,1,5);
    t = linspace(0,tmeasure,size(input_history,1));
    plot(t,input_history(:),'-c','DisplayName','Current Trajectory')
    xlabel('t[s]');
    ylabel('theta dot [deg/s]');
    drawnow
end


function pendel = system_dynamics(t, x ,u, dW)
% System parameters
m = .5;  % pendulum mass (kg)
M = 1;  % car mass  (kg)
g = 9.81;   % gravity force (m/s^2)
l = 0.5;    % pendulum length (m)
fx = 0;

                pendel = [x(2) + dW(1);
        (l*m*sin(x(3))*x(4)^2 + u(1) + fx*x(2) - g*m*cos(x(3))*sin(x(3)))/(M + m*(sin(x(3))^2)) + dW(2);
        x(4)+ dW(3);
        (-m*l*x(4)^2*sin(x(3))*cos(x(3))-u(1)*cos(x(3))-fx*x(2)*cos(x(3))+(g*sin(x(3)))*(m+M))/((sin(x(3))^2)*m*l + M*l)+ dW(4)]; 


end