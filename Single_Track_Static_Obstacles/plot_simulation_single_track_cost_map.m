clc

clear all;
close all;
warning off
%

% decided which simulation to plot
% one without any obtacles: with_obstacles=0
% one with static obtacles: with_obstacles=1

with_obstacles = 1;

if with_obstacles == 0
    fname = '/home/unicornfarm/Documents/DVD/Single_Track_Static_Obs/simulations/single_track_round_simulation_2.mat';
    data = open(fname);
    states_history = data.states_history;
    control = data.u_opt;
    % obstacles = data.obstacle;
    S_save = data.S_save;
    trajectory_history = data.trajectory_history;
else
    fname = '/home/unicornfarm/Documents/DVD/Single_Track_Static_Obs/simulations/single_track_round_simulation_obstacle_2.mat';
    data = open(fname);
    states_history = data.states_history;
    control = data.u_opt;
    obstacles = data.obstacle;
    S_save = data.S_save;
    fname = '/home/unicornfarm/Documents/DVD/Single_Track_Static_Obs/simulations/single_track_round_simulation_obstacle_2_traj.mat';
    data = open(fname);
    trajectory_history = data.trajectory_history;
end

% states_history = data.states_history(1:337,:);
% control = data.u_opt(1:337,:);
% trajectory_history = data.trajectory_history;
% S_save = data.S_save(1:337,:);

% obstacles_ = data.obstacle;



[map, cost_map, obstacle] = create_map();
if with_obstacles == 1
    obstacles = [1 51;2 51;3 51;4 51;5 51;6 51;7 51;12 71;11 71;10 71;9 71;8 71;7 71;6 71]
    setOccupancy(map,obstacles,1);
end

%% Control plots
% figure
% t = linspace(0,size(control,1),size(control,1));
% plot(t,control(:,1))
% axis tight
% title(['Steering Control'],'Interpreter','latex');
% itm_formatfig('LatexNarrow')
%
% xlabel('$t$[s]','Interpreter','latex');
% ylabel('Steering [rad]','Interpreter','latex');
% legend({'Steering Control'},'Interpreter','latex')
%
%  print('-depsc', 'control_steering.eps')
% print('-dpng', 'control_steering.png')
%
% figure
% t = linspace(0,size(control,1),size(control,1));
% plot(t,control(:,2))
% axis tight
% title(['Breaking Force'],'Interpreter','latex');
% itm_formatfig('LatexNarrow')
%
% xlabel('$t$[s]','Interpreter','latex');
% ylabel('Breaking Force [N]','Interpreter','latex');
% legend({'Breaking Force'},'Interpreter','latex')
%  print('-depsc', 'control_breakingforce_2.eps')
% print('-dpng', 'control_breakingforce_2.png')
%
% figure
% t = linspace(0,size(control,1),size(control,1));
% plot(t,control(:,3))
% axis tight
% title(['Break Distribution'],'Interpreter','latex');
% itm_formatfig('LatexNarrow')
%
% xlabel('$t$[s]','Interpreter','latex');
% ylabel('Break Distribution','Interpreter','latex');
% legend({'Break Distribution'},'Interpreter','latex')
%  print('-depsc', 'control_breakingdistribution_2.eps')
% print('-dpng', 'control_breakingdistribution_2.png')
%
%
% figure
% t = linspace(0,size(control,1),size(control,1));
% plot(t,control(:,4))
% axis tight
% title(['Gas Pedal'],'Interpreter','latex');
% itm_formatfig('LatexNarrow')
%
% xlabel('$t$[s]','Interpreter','latex');
% ylabel('Gas Pedal','Interpreter','latex');
% legend({'Gas Pedal'},'Interpreter','latex')
%  print('-depsc', 'control_gaspedal-2.eps')
% print('-dpng', 'control_gaspedal-2.png')

%% Cost plots
% figure
% t =linspace(0,size(states_history,1),size(states_history,1))/10;
% for i=1:1:size(states_history,1)
% grid_idx = world2grid(map,[states_history(i,1) states_history(i,2)]);
% cost_grid_square = cost_map(grid_idx(1),grid_idx(2));
% Q = diag([0,0,50,0,0,0,0,0,0,0]);
% x_desired = [0 0 30 0 0 0 0 0 0 0];
% J_grid(i) =1*cost_grid_square;
%
% J_vel(i) = (states_history(i,:)-x_desired)*Q*(states_history(i,:)-x_desired)';
% Q = diag([0,0,0,5,0,0,0,0,0,0]);
% x_desired = [0 0 30 0 0 0 0 0 0 0];
% J_slip(i) = (states_history(i,:)-x_desired)*Q*(states_history(i,:)-x_desired)';
% end
% J_grid(332)=J_grid(331);
% plot(t,J_grid)
% axis tight
% % title(['Grid cost'],'Interpreter','latex');
% itm_formatfig('LatexNarrow')
%
% xlabel('$t$ [s]');
% ylabel('grid cost');
% legend({'Grid cost'},'Interpreter','latex')
%  print('-depsc', 'gridcost.eps')
% print('-dpng', 'gridcost.png')
% %
% figure
% plot(t,J_vel)
% axis tight
% title(['Velocity cost'],'Interpreter','latex');
% itm_formatfig('LatexNarrow')
%
% xlabel('$t$[s]','Interpreter','latex');
% ylabel('Velocity cost','Interpreter','latex');
% legend({'Velocity cost'},'Interpreter','latex')
%  print('-depsc', 'velocitycost.eps')
% print('-dpng', 'velocitycost.png')
%
% figure
% plot(t,J_vel)
% hold on
% plot(t,J_grid)
% axis tight
% title(['Velocity cost'],'Interpreter','latex');
% itm_formatfig('LatexNarrow')
%
% xlabel('$t$[s]','Interpreter','latex');
% ylabel('Velocity cost','Interpreter','latex');
% legend({'Velocity cost'},'Interpreter','latex')
%  print('-depsc', 'velocitycost.eps')
% print('-dpng', 'velocitycost.png')
%
% figure
% t = linspace(0,size(states_history,1),size(states_history,1))/10;
% plot(t,states_history(:,3))
% axis tight
% % title(['Velocity'],'Interpreter','latex');
% itm_formatfig('LatexNarrow')
%
% xlabel('$t$ [s]');
% ylabel('velocity [m/s]');
% legend({'Velocity'},'Interpreter','latex')
%  print('-depsc', 'cost_map_velocity.eps')
% print('-dpng', 'cost_map_velocity.png')
%
% figure
% plot(t,states_history(:,4))
%
%

%% Animation

% setup for saving the pictures for the video
filepath = [pwd,filesep,'myVideo',filesep]; % folder for pngs
filetype_ = 'png';
flagPrintPngs = false;    % flag to apply the print command
dpi_ = 300;               % resolution to print figures {90}

if flagPrintPngs
    if exist(filepath,'dir')
        %         Delete all files of this type in the specified directory
        if ~isempty(dir([filepath,'*.',filetype_]))
            delete([filepath,'*.',filetype_]);
        end
    else
        mkdir(filepath)
    end
end

h_videoFig = figure(7);
counter = 0;

% begin animation
for t=2:1:size(states_history,1)-1
    
    figure(7)
    hold off
    
    show(map);
    hold on
    grid on
    title('')
    S = S_save(t,:);
    save_trajectory = trajectory_history(:,:,:,t);
    
    % get color of each trajectory based on the cost
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
        temperature(i) = find(temperature_range < round(S_norm(i),2)+0.01 & temperature_range > round(S_norm(i),2)-0.01,1);
        color(i,:,:) = newcolormap(temperature(i),:,:);
        
    end
    
    % plot trajectories with respective color
    cost_print = 9999999999999999999999;
    for k=1:1:size(save_trajectory,3)
        plot(save_trajectory(:,1,k),save_trajectory(:,2,k),'Color', color(k,:,:))
    end
    
    %  plot states
    plot(states_history(1:t,1),states_history(1:t,2),'-b','DisplayName','Current Trajectory')
    
    % plot single-track
    l_f=1.19016;
    l_r=1.37484;
    l=l_f+l_r;
    R=0.302;
    x_ = states_history(t,1); y_ = states_history(t,2);
    
    car = plot([x_-l_r  x_+l_f  x_+l_f  x_-l_r  x_-l_r], [y_-0.5  y_-0.5  y_+0.5  y_+0.5  y_-0.5],'LineWidth',2);
    rotate(car, [0 0 1], rad2deg(states_history(t,5)),[x_ y_ 0]);
    wheel_1 = plot([x_+l_f-R  x_+l_f+R  x_+l_f+R  x_+l_f-R  x_+l_f-R], [y_+0.25  y_+0.25  y_-0.25  y_-0.25  y_+0.25],'LineWidth',2,'Color','r');
    wheel_2 = plot([x_-l_r-R  x_-l_r+R  x_-l_r+R  x_-l_r-R  x_-l_r-R], [y_+0.25  y_+0.25  y_-0.25  y_-0.25  y_+0.25],'LineWidth',2,'Color','b');
    % wheel_3 = plot([x_+l_f-R  x_+l_f+R  x_+l_f+R  x_+l_f-R  x_+l_f-R], [y_-1.10/2  y_-1.10/2  y_-1.10/2+0.3  y_-1.10/2+0.3  y_-1.10/2],'LineWidth',2,'Color','r');
    % wheel_4 = plot([x_-l_r-R  x_-l_r+R  x_-l_r+R  x_-l_r-R  x_-l_r-R], [y_-1.10/2  y_-1.10/2  y_-1.10/2+0.3  y_-1.10/2+0.3  y_-1.10/2],'LineWidth',2,'Color','b');
    
    % texts
    txt3 =  [ 'vel [m/s]: ' num2str(states_history(t,3))];
    t3 = text(1,2.5,txt3,'FontSize',10,'Color','red');
    txt4 =  [ 'time [s]: ' num2str((t/10)-0.1)];
    t4 = text(1,5.5,txt4,'FontSize',10,'Color','red');
    
    rotate(wheel_1, [0 0 1], rad2deg(states_history(t,5)),[x_ y_ 0]);
    rotate(wheel_2, [0 0 1], rad2deg(states_history(t,5)),[x_ y_ 0]);
    rotate(wheel_1, [0 0 1], rad2deg(control(1,1)),[x_+l_f*cos(states_history(t,5)) y_+l_f*sin(states_history(t,5)) 0]);
    % rotate(wheel_3, [0 0 1], rad2deg(states_history(t,5)),[x_ y_ 0]);
    % rotate(wheel_4, [0 0 1], rad2deg(states_history(t,5)),[x_ y_ 0]);
    % rotate(wheel_3, [0 0 1], rad2deg(control(1,1)),[x_+l_f*cos(states_history(t,5)) y_+l_f*sin(states_history(t,5)) 0]);
    % axis([0 50 0 100])
    colorbar('off')
    % plot(path(:,1), path(:,2))
    % text(x_ +1 ,y_ + 3,num2str(states_history(t,3)));
    drawnow
    % title(['Cost Map'],'Interpreter','latex');
    xlabel('x [m]');
    ylabel('y [m]');
    % itm_formatfig(2)
    set(gca,'XTick',0:15:100,'YTick',0:15:100);
    if flagPrintPngs
        print(h_videoFig,sprintf('%sframe_%05d',filepath,counter),['-d',filetype_],'-loose',['-r0',num2str(dpi_)])
        counter = counter +1;
    end
end
