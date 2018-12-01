%% This script reads the simulations results
% clc
% clear all;
% close all;
% warning off


simulation_idx = 0;
fname = sprintf('/home/unicornfarm/Documents/Studienarbeit/Cases/Decenralzed_multi_agents/simulations/simulation_multi_traj_foll.mat');
data = open(fname);

u_opt_ = data.u_opt;
u_opt2_ = data.u_opt2;
u_opt3_ = data.u_opt3;
states_history_ = data.states_history;
states_history2_ = data.states_history2;
states_history3_ = data.states_history3;
trajectory_history_ = data.trajectory_history;
trajectory_history2_ = data.trajectory_history2;
trajectory_history3_ = data.trajectory_history3;
S_history_ = data.S_save;
S_history2_ = data.S_save2;
S_history3_ = data.S_save3;

y_= linspace(-3,6,400)';
x_ = zeros(1,400)';
path = [x_ y_];
controller = robotics.PurePursuit;
controller.Waypoints = path;
controller.LookaheadDistance = 0.5;

save_center = [];
% filepath = [pwd,filesep,'myVideo',filesep]; % folder for pngs
% filetype_ = 'png';
% flagPrintPngs = true;    % flag to apply the print command
% dpi_ = 90;               % resolution to print figures {90}
% 
% if flagPrintPngs
%     if exist(filepath,'dir')
% %         Delete all files of this type in the specified directory
%         if ~isempty(dir([filepath,'*.',filetype_]))
%             delete([filepath,'*.',filetype_]);
%         end
%     else
%         mkdir(filepath)
%     end
% end
% 
% h_videoFig = figure(7);
% 
% set(gcf,'position',[0 0 1280 720])  % 1280:720px --> 16:9 ratio, full-HD
% set(gcf,'PaperPositionMode','auto') % take print-pixel from window-pixel
% set(gca, 'Position', get(gca, 'OuterPosition') -get(gca, 'TightInset') * 0.00001*[-1 0 1 0; 0 -1 0 1; -1 0 1 0;0 -1 0 1]);
% 
% counter = 0;
for i=1:1:size(states_history_ ,1)


states_history = states_history_(i,:);
states_history2 = states_history2_(i,:);
states_history3 = states_history3_(i,:);
save_trajectory = trajectory_history_(:,:,:,i);
save_trajectory2 = trajectory_history2_(:,:,:,i);
save_trajectory3 = trajectory_history3_(:,:,:,i);
% S = S_history_(i,:);
% S2 = S_history2_(i,:);
% S3 = S_history3_(i,:);


figure(7)
cla
% show(map);
% hold on

% set(gca,'XTick',0:1:10,'YTick',0:1:10)
% colormap(gca,hot())

% colormap_ = flipud(hot());
% colormap_ = flipud(autumn());
% r = 0.3; % magnitude (length) of arrow to plot


% A = [1:size(colormap_,1)/100:size(colormap_,1)]';
% B = [1:1:size(colormap_,1)]';
% 
% newcolormap = interp1(B, colormap_ ,A);
% S_norm = (S - min(S))/(max(S) - min(S));
% 
% temperature_range = [0:1/98:1];
% temperature = zeros(1, size(S_norm,2));
% color = zeros(size(S_norm,2),3,1);
% 
% for i=1:1:size(S_norm,2)
%     temperature(i) = find(temperature_range < round(S_norm(i),2)+0.01 & temperature_range > round(S_norm(i),2)-0.01,1);
%     color(i,:,:) = newcolormap(temperature(i),:,:);
%     
% end
% Plots the states
hold on
plot(states_history_(1:i,1),states_history_(1:i,2),'-b','DisplayName','Current Trajectory 1')
plot(states_history2_(1:i,1),states_history2_(1:i,2),'-g','DisplayName','Current Trajectory 2')
plot(states_history3_(1:i,1),states_history3_(1:i,2),'-r','DisplayName','Current Trajectory 3') 

% get the color of each trajectory based on the cost
% for k=1:1:size(save_trajectory,3)
%     plot(save_trajectory(:,1,k),save_trajectory(:,2,k),'Color', color(k,:,:))
% end
% for k=1:1:size(save_trajectory,3)
%     plot(save_trajectory(:,1,k),save_trajectory(:,2,k))
% end


% A = [1:size(colormap_,1)/100:size(colormap_,1)]';
% B = [1:1:size(colormap_,1)]';
% 
% newcolormap = interp1(B, colormap_ ,A);
% S_norm = (S2 - min(S2))/(max(S2) - min(S2));
% 
% temperature_range = [0:1/98:1];
% temperature = zeros(1, size(S_norm,2));
% color = zeros(size(S_norm,2),3,1);
% 
% for i=1:1:size(S_norm,2)
%     temperature(i) = find(temperature_range < round(S_norm(i),2)+0.01 & temperature_range > round(S_norm(i),2)-0.01,1);
%     color(i,:,:) = newcolormap(temperature(i),:,:);
%     
% end
% plot(states_history(:,1),x_desiredstates_history(:,2),'-b','DisplayName','Current Trajectory')
% for k=1:1:size(save_trajectory2,3)
%     plot(save_trajectory2(:,1,k),save_trajectory2(:,2,k))
% %     plot(save_trajectory2(:,1,k),save_trajectory2(:,2,k),'Color', color(k,:,:))
% end


% A = [1:size(colormap_,1)/100:size(colormap_,1)]';
% B = [1:1:size(colormap_,1)]';
% 
% newcolormap = interp1(B, colormap_ ,A);
% S_norm = (S3 - min(S3))/(max(S3) - min(S3));
% 
% temperature_range = [0:1/98:1];
% temperature = zeros(1, size(S_norm,2));
% color = zeros(size(S_norm,2),3,1);
% 
% for i=1:1:size(S_norm,2)
%     temperature(i) = find(temperature_range < round(S_norm(i),2)+0.01 & temperature_range > round(S_norm(i),2)-0.01,1);
%     color(i,:,:) = newcolormap(temperature(i),:,:);
%     
% end

% for k=1:1:size(save_trajectory3,3)
%     plot(save_trajectory3(:,1,k),save_trajectory3(:,2,k))
% %     plot(save_trajectory3(:,1,k),save_trajectory3(:,2,k),'Color', color(k,:,:))
% end

% Plot the robots
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
    
% plot the desired trajectory
plot((1:5:i)/50,2*sin((1:5:i)/50),'c','LineWidth',2);

c_x = (1/3)*(states_history(end,1) + states_history2(end,1) + states_history3(end,1));
c_y = (1/3)*(states_history(end,2) + states_history2(end,2) + states_history3(end,2));
rad =0.05;
angs = 0:pi/10:2*pi;
y = c_x + rad*sin(angs);
x = c_y + rad*cos(angs);
save_center = [save_center;c_x c_y];
plot(save_center(:,1),save_center(:,2),'k','LineWidth',0.5);

grid on


colormap(gca,flipud(hot()))



xlabel('x [m]');
ylabel('y [m]');
axis equal
axis([-3.5 8 -3.6 3.6])
drawnow

% itm_formatfig('LatexNarrow')

% legend({'No filter','Filter on the control','Filter on the weights'},'Interpreter','latex')
% for idx = 1:1
%     if flagPrintPngs
% 
%         print(h_videoFig,sprintf('%sframe_%05d',filepath,counter),['-d',filetype_],'-loose',['-r0',num2str(dpi_)])
% %             print([h_videoFig,['-d',filetype_],...
%            % '-loose',['-r0',num2str(dpi_)],sprintf('%sframe_%04d',filepath,idx));
%     %         Print the frames to filepath
%     %         print(['-f',num2str(h_videoFig,'%18.16f')],['-d',filetype_],...
%     %         '-loose',['-r0',num2str(dpi_)],sprintf('%sframe_%04d',filepath,idx));
%     counter = counter +1;
%         else
%             pause(0.0005) % hold the figure for some mili-seconds
%         end
%  end
% print('-depsc', 'multi_4.eps')
% print('-dpng', 'multi_4.png')
end
