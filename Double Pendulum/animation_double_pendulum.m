
close all
clear all
clc


fname = sprintf('/home/unicornfarm/Documents/Studienarbeit/Cases/Double Pendulum/simulations/simulation_test.mat');
data = open(fname);
states_history= data.states_history;


    m0 = 1.5;   %mass cart
    m1 = 0.5;   %mass link 1
    m2 = 0.75;  %mass link 2
    L1 = 0.5;   %lenght link 1
    L2 = 0.75;  %lenght link 2



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
%     title(['Inverted Pendulum Time: ' num2str(i*sys.dt,'%.2f') 's'])
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
    pendel_obj = line([states_history(i,1),states_history(i,1) + sin(states_history(i,2))*L1],[car_height+0.05, car_height+ 0.05 + cos(states_history(i,2))*L1], 'LineWidth',2);
    pendel_obj2 = line([states_history(i,1) + sin(states_history(i,2))*L1 ,states_history(i,1) + sin(states_history(i,2))*L1 + sin(states_history(i,3))*L2],[car_height+0.05 + cos(states_history(i,2))*L1, car_height+ 0.05 + cos(states_history(i,2))*L1 + cos(states_history(i,3))*L2], 'LineWidth',2);

    hold on
    
    
    
    
%     sz = linspace(0.1,20,size(states_history(:,1),1));
%     sz = ones(1,size(states_history(i,1),1))*2;
    % Draw the end of the pendulum trajectory and ball
    ball =rectangle('Position',[(states_history(i,1)+sin(states_history(i,2))*L1+sin(states_history(i,3))*L2-ball_diameter/2) car_height+0.05+cos(states_history(i,2))*L1+cos(states_history(i,3))*L2-ball_diameter/2 ball_diameter ball_diameter],'Curvature',[1 1]);
    ball.EdgeColor = 'r';
    ball.FaceColor = [1 0 0];
    s = scatter(states_history(i,1) + sin(states_history(i,2))*L1+ sin(states_history(i,3))*L2,car_height+ 0.05 + cos(states_history(i,2))*L1+ cos(states_history(i,3))*L2,2);    
    s.MarkerEdgeColor = 'r';
    s.MarkerFaceColor = [1.0 0.0 0.0];
    hold on
    % Draw the floor
    plot([min(min(states_history(:,1)),-0.55)-0.5 max(max(states_history(:,1)),0.55)+0.5],[0 0],'b','LineWidth',2)
    drawnow
    pause(0.1);
    if i<size(states_history,1)-1
        delete(pendel_obj);
        delete(pendel_obj2);
        delete(car)
        delete(wheel_left)
        delete(wheel_right)
        delete(ball)
    end
end


