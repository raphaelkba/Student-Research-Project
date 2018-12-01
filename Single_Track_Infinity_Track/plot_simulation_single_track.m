%% Plots and animation for the single-track simulations in a infinity/eight shaped track
% clc
% clear all;
% close all;
% warning off

%% Open the data
fname = 'simulation_inf_1.mat';
data = open(fname);
states_history = data.states_history;
control = data.u_opt;
trajectory_history = data.trajectory_history;
% S_save = data.S_save;
path = data.path;


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

% set(gcf,'position',[0 0 1280 720])  % 1280:720px --> 16:9 ratio, full-HD
% set(gcf,'PaperPositionMode','auto') % take print-pixel from window-pixel
% set(gca, 'Position', get(gca, 'OuterPosition') -get(gca, 'TightInset') * 0.00001*[-1 0 1 0; 0 -1 0 1; -1 0 1 0;0 -1 0 1]);

%% Initialize the animation the animation
% for t=1:1:size(states_history,1)-1
    for t=1:1:10

    figure(7)
    subplot(1,2,1)  
    hold off
    
    %% plot the track
    % showt_4_r=[-30*ones(150,1) (linspace(250,400,150))']; % right racetrack boundary, segment 4
    t_4_l=[-35*ones(150,1) (linspace(250,400,150))']; % left racetrack boundary, segment 4
    t_5_r=[(linspace(-30,20,200))' sqrt(625-((linspace(-30,20,200))'+5).^2)+400]; % right racetrack boundary, segment 5
    t_5_l=[(linspace(-35,25,200))' sqrt(900-((linspace(-35,25,200))'+5).^2)+400]; % left racetrack boundary, segment 5
    t_6_r=[20*ones(100,1) (linspace(400,250,100))']; % right racetrack boundary, segment 6
    t_6_l=[25*ones(100,1) (linspace(400,250,100))']; % left racetrack boundary, segment 6
    t_7_r=[-(linspace(-30,20,200))'-10 -sqrt(625-((linspace(-30,20,200))'+5).^2)+250]; % right racetrack boundary, segment 5
    t_7_l=[-(linspace(-35,25,200))'-10 -sqrt(900-((linspace(-35,25,200))'+5).^2)+250]; % left racetrack boundary, segment 5
    
    % t_r=[t_4_r ; t_5_r ; t_6_r;t_7_r ];
    % t_l=[t_4_l ; t_5_l ; t_6_l;t_7_l];
    t_r=[t_5_l; flip(t_7_r );t_5_l(1,:)];
    t_l=[t_5_r  ;flip(t_7_l);t_5_r(1,:);];
    % load('curve_bez.mat', 'curve_bez');
    % figure('Name','racetrack','NumberTitle','off','Toolbar','figure','MenuBar','none','OuterPosition',[0 -500 460 1100]) % creates window for plot
    % hold on % allow for multiple plot commands within one figure
%     axis equal % eqal axis scaling
%     axis([-50 70 -50 450]) % plot height and width
    h1 = plot(t_r(:,1),t_r(:,2)-200,'k'); % plot right racetrack boundary
    hold on
    plot(t_l(:,1),t_l(:,2)-200,'k'); % plot left racetrack boundary
    
    h2 = plot(path(:,1),path(:,2)-200);
    S = S_save(t,:);
    hold on
    grid on
    save_trajectory = trajectory_history(:,:,:,t);
    % set(gca,'XTick',0:1:size_x,'YTick',0:1:size_y);
    
    colormap = winter;
    c = colorbar;
    c.Label.String = 'cost of the sampled trajectory';
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
    
    cost_print = 9999999999999999999999;
    for k=1:1:size(save_trajectory,3)
        
        plot(save_trajectory(:,1,k),save_trajectory(:,2,k)-200,'Color', color(k,:,:))
    end
    
    
    x_ = states_history(t,1); y_ = states_history(t,2)-200;
    
    h3 = plot(states_history(1:t,1),states_history(1:t,2)-200,'-r');
    
    l_f=1.19016;
    l_r=1.37484;
    l=l_f+l_r;
    R=0.302;
    
    
    car = plot([x_-l_r  x_+l_f  x_+l_f  x_-l_r  x_-l_r], [y_-1  y_-1  y_+1  y_+1  y_-1],'LineWidth',2);
    rotate(car, [0 0 1], rad2deg(states_history(t,5)),[x_ y_ 0]);
    wheel_1 = plot([x_+l_f-R  x_+l_f+R  x_+l_f+R  x_+l_f-R  x_+l_f-R], [y_+0.25  y_+0.25  y_-0.25  y_-0.25  y_+0.25],'LineWidth',2,'Color','r');
    wheel_2 = plot([x_-l_r-R  x_-l_r+R  x_-l_r+R  x_-l_r-R  x_-l_r-R], [y_+0.25  y_+0.25  y_-0.25  y_-0.25  y_+0.25],'LineWidth',2,'Color','b');
    % wheel_3 = plot([x_+l_f-R  x_+l_f+R  x_+l_f+R  x_+l_f-R  x_+l_f-R], [y_-1.10/2  y_-1.10/2  y_-1.10/2+0.3  y_-1.10/2+0.3  y_-1.10/2],'LineWidth',2,'Color','r');
    % wheel_4 = plot([x_-l_r-R  x_-l_r+R  x_-l_r+R  x_-l_r-R  x_-l_r-R], [y_-1.10/2  y_-1.10/2  y_-1.10/2+0.3  y_-1.10/2+0.3  y_-1.10/2],'LineWidth',2,'Color','b');
    
    
    rotate(wheel_1, [0 0 1], rad2deg(states_history(t,5)),[x_ y_ 0]);
    rotate(wheel_2, [0 0 1], rad2deg(states_history(t,5)),[x_ y_ 0]);
    rotate(wheel_1, [0 0 1], rad2deg(control(1,1)),[x_+l_f*cos(states_history(t,5)) y_+l_f*sin(states_history(t,5)) 0]);
    % rotate(wheel_3, [0 0 1], rad2deg(states_history(t,5)),[x_ y_ 0]);
    % rotate(wheel_4, [0 0 1], rad2deg(states_history(t,5)),[x_ y_ 0]);
    % rotate(wheel_3, [0 0 1], rad2deg(control(1,1)),[x_+l_f*cos(states_history(t,5)) y_+l_f*sin(states_history(t,5)) 0]);
    colorbar('off')
%     camroll(-90)
    states_history(1,3) =0;
    txt3 =  [ 'Vel [m/s]: ' num2str(states_history(t,3))];
    t3 = text(-44,10,txt3,'FontSize',10,'Color','red');
    axis equal
    axis([-50 40 0 250])
    xlabel('x [m]');
    ylabel('y [m]');
%     axis fill
%     itm_formatfig('LatexNarrow')
%     itm_formatfig('LatexWide')

    % legend([h1 h2 h3],{'Track Boundaries','Desired Trajectory','Driven Trajectory'})
    % plot(path(:,1), path(:,2))
    % text(x_ +1 ,y_ + 3,num2str(states_history(t,3)));
    
    
    subplot(1,2,2)  
    
        hold off
    % showt_4_r=[-30*ones(150,1) (linspace(250,400,150))']; % right racetrack boundary, segment 4
    t_4_l=[-35*ones(150,1) (linspace(250,400,150))']; % left racetrack boundary, segment 4
    t_5_r=[(linspace(-30,20,200))' sqrt(625-((linspace(-30,20,200))'+5).^2)+400]; % right racetrack boundary, segment 5
    t_5_l=[(linspace(-35,25,200))' sqrt(900-((linspace(-35,25,200))'+5).^2)+400]; % left racetrack boundary, segment 5
    t_6_r=[20*ones(100,1) (linspace(400,250,100))']; % right racetrack boundary, segment 6
    t_6_l=[25*ones(100,1) (linspace(400,250,100))']; % left racetrack boundary, segment 6
    t_7_r=[-(linspace(-30,20,200))'-10 -sqrt(625-((linspace(-30,20,200))'+5).^2)+250]; % right racetrack boundary, segment 5
    t_7_l=[-(linspace(-35,25,200))'-10 -sqrt(900-((linspace(-35,25,200))'+5).^2)+250]; % left racetrack boundary, segment 5
    
    % t_r=[t_4_r ; t_5_r ; t_6_r;t_7_r ];
    % t_l=[t_4_l ; t_5_l ; t_6_l;t_7_l];
    t_r=[t_5_l; flip(t_7_r );t_5_l(1,:)];
    t_l=[t_5_r  ;flip(t_7_l);t_5_r(1,:);];
    % load('curve_bez.mat', 'curve_bez');
    % figure('Name','racetrack','NumberTitle','off','Toolbar','figure','MenuBar','none','OuterPosition',[0 -500 460 1100]) % creates window for plot
    % hold on % allow for multiple plot commands within one figure
%     axis equal % eqal axis scaling
%     axis([-50 70 -50 450]) % plot height and width
    h1 = plot(t_r(:,1),t_r(:,2)-200,'k'); % plot right racetrack boundary
    hold on
    plot(t_l(:,1),t_l(:,2)-200,'k'); % plot left racetrack boundary
    
    h2 = plot(path(:,1),path(:,2)-200);
    S = S_save(t,:);
    hold on
    grid on
    save_trajectory = trajectory_history(:,:,:,t);
    % set(gca,'XTick',0:1:size_x,'YTick',0:1:size_y);
    
    colormap = winter;
    c = colorbar;
    c.Label.String = 'cost of the sampled trajectory';
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
    
%     cost_print = 9999999999999999999999;
%     for k=1:1:size(save_trajectory,3)
%         
%         plot(save_trajectory(:,1,k),save_trajectory(:,2,k)-200,'Color', color(k,:,:))
%     end
    
    
    x_ = states_history(t,1); y_ = states_history(t,2)-200;
    
    h3 = plot(states_history(1:t,1),states_history(1:t,2)-200,'-r');
    
    l_f=1.19016;
    l_r=1.37484;
    l=l_f+l_r;
    R=0.302;
    
    
    car = plot([x_-l_r  x_+l_f  x_+l_f  x_-l_r  x_-l_r], [y_-.5  y_-.5  y_+.5  y_+.5  y_-.5],'LineWidth',2);
    rotate(car, [0 0 1], rad2deg(states_history(t,5)),[x_ y_ 0]);
    wheel_1 = plot([x_+l_f-R  x_+l_f+R  x_+l_f+R  x_+l_f-R  x_+l_f-R], [y_+0.25  y_+0.25  y_-0.25  y_-0.25  y_+0.25],'LineWidth',2,'Color','r');
    wheel_2 = plot([x_-l_r-R  x_-l_r+R  x_-l_r+R  x_-l_r-R  x_-l_r-R], [y_+0.25  y_+0.25  y_-0.25  y_-0.25  y_+0.25],'LineWidth',2,'Color','b');
    % wheel_3 = plot([x_+l_f-R  x_+l_f+R  x_+l_f+R  x_+l_f-R  x_+l_f-R], [y_-1.10/2  y_-1.10/2  y_-1.10/2+0.3  y_-1.10/2+0.3  y_-1.10/2],'LineWidth',2,'Color','r');
    % wheel_4 = plot([x_-l_r-R  x_-l_r+R  x_-l_r+R  x_-l_r-R  x_-l_r-R], [y_-1.10/2  y_-1.10/2  y_-1.10/2+0.3  y_-1.10/2+0.3  y_-1.10/2],'LineWidth',2,'Color','b');
    
    
    rotate(wheel_1, [0 0 1], rad2deg(states_history(t,5)),[x_ y_ 0]);
    rotate(wheel_2, [0 0 1], rad2deg(states_history(t,5)),[x_ y_ 0]);
        rotate(wheel_1, [0 0 1], rad2deg(control(t,1)),[x_+l_f*cos(states_history(t,5)) y_+l_f*sin(states_history(t,5)) 0]);

%     rotate(wheel_1, [0 0 1], rad2deg(control(1,1)),[x_+l_f*cos(states_history(t,5)) y_+l_f*sin(states_history(t,5)) 0]);
    % rotate(wheel_3, [0 0 1], rad2deg(states_history(t,5)),[x_ y_ 0]);
    % rotate(wheel_4, [0 0 1], rad2deg(states_history(t,5)),[x_ y_ 0]);
    % rotate(wheel_3, [0 0 1], rad2deg(control(1,1)),[x_+l_f*cos(states_history(t,5)) y_+l_f*sin(states_history(t,5)) 0]);
    colorbar('off')
%     camroll(-90)
    states_history(1,3) =0;
%     txt3 =  [ 'Vel [m/s]: ' num2str(states_history(t,3))];
%     t3 = text(-42,180,txt3,'FontSize',10,'Color','red');
    axis equal
    axis([states_history(t,1)-5 states_history(t,1)+5 states_history(t,2)-5-200 states_history(t,2)+5-200])
    xlabel('x [m]');
    ylabel('y [m]');
    
    
    
    
    
    drawnow
%     print('-depsc', 'single_track_infinity_5_narrow.eps')
%     print('-dpng', 'single_track_infinity_5_narrow.png')
    % print('-depsc', 'single_track_infinity_4_wide.eps')
%     print('-dpng', 'single_track_infinity_4_wide.png')
     for idx = 1:1
    if flagPrintPngs

        print(h_videoFig,sprintf('%sframe_%05d',filepath,counter),['-d',filetype_],'-loose',['-r0',num2str(dpi_)])
%             print([h_videoFig,['-d',filetype_],...
           % '-loose',['-r0',num2str(dpi_)],sprintf('%sframe_%04d',filepath,idx));
    %         Print the frames to filepath
    %         print(['-f',num2str(h_videoFig,'%18.16f')],['-d',filetype_],...
    %         '-loose',['-r0',num2str(dpi_)],sprintf('%sframe_%04d',filepath,idx));
    counter = counter +1;
        else
            pause(0.0005) % hold the figure for some mili-seconds
        end
 end
 end

%% Control plots
figure(9)
t = linspace(0,size(control,1),size(control,1));
plot(t,control(:,1))
axis tight
itm_formatfig('LatexWide')
legend({'Steering Angle'})

% figure(10)
% step(t,control(:,2))
% axis tight
% itm_formatfig('LatexWide')
% legend({'Breaking Force'})

figure(11)
plot(t,control(:,3))
axis tight
itm_formatfig('LatexWide')
legend({'Breaking Distribution'})

figure(12)
plot(t,control(:,4))
axis tight
itm_formatfig('LatexWide')
legend({'Gas Pedal'})

% States plots
figure
t = linspace(0,size(states_history,1),size(states_history,1))/10;
plot(t,states_history(:,3))
axis tight
% title(['Velocity'],'Interpreter','latex');
% itm_formatfig('LatexNarrow')
% legend({'velocity'})
xlabel('t [s]');
ylabel('velocity [m/s]');
drawnow()
% print('-depsc', 'single_track_infinity_velocity_sim1_narrow.eps')
print('-dpng', 'single_track_infinity_velocity_sim1_narrow.png')
% 
% figure
% t = linspace(0,size(states_history,1),size(states_history,1));
% plot(t,states_history(:,4))
% axis tight
% title(['Slip Velocity'],'Interpreter','latex');
% itm_formatfig('LatexWide')
% xlabel('$t$[s]','Interpreter','latex');
% ylabel('Slip velocity [m/s]','Interpreter','latex');
% legend({'Slip velocity'})
% drawnow()
% print('-depsc', 'single_track_infinity_slipvelocity_sim2.eps')
% print('-dpng', 'single_track_infinity_slipvelocity_sim2.png')
% %% Cost
x_desired = [0 0 30 0 0 0 0 0 0 0];
Q = diag([0,0,0.5,100,0,0,0,0,0,0]);
t_4_r=[-30*ones(150,1) (linspace(250,400,150))']; % right racetrack boundary, segment 4
t_4_l=[-35*ones(150,1) (linspace(250,400,150))']; % left racetrack boundary, segment 4
t_5_r=[(linspace(-30,20,200))' sqrt(625-((linspace(-30,20,200))'+5).^2)+400]; % right racetrack boundary, segment 5
t_5_l=[(linspace(-35,25,200))' sqrt(900-((linspace(-35,25,200))'+5).^2)+400]; % left racetrack boundary, segment 5
t_6_r=[20*ones(100,1) (linspace(400,250,100))']; % right racetrack boundary, segment 6
t_6_l=[25*ones(100,1) (linspace(400,250,100))']; % left racetrack boundary, segment 6
t_7_r=[-(linspace(-30,20,200))'-10 -sqrt(625-((linspace(-30,20,200))'+5).^2)+250]; % right racetrack boundary, segment 5
t_7_l=[-(linspace(-35,25,200))'-10 -sqrt(900-((linspace(-35,25,200))'+5).^2)+250]; % left racetrack boundary, segment 5

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
% t_r=[a(end,:);t_5_l(1,:)];
% t_l=[b(end,:);t_5_r(1,:)];
% t_r=[t_5_l(end,:);a(1,:)];
% t_l=[t_5_r(end,:);b(1,:)];
c1_r=[t_5_l];
c1_l=[t_5_r];
c2_r=[flip(t_7_r )];
c2_l=[flip(t_7_l)];
t_r_x=t_r(1:1:end,1); % x coordinate of right racetrack boundary
t_r_y=t_r(1:1:end,2); % y coordinate of right racetrack boundary
t_l_x=t_l(1:1:end,1); % x coordinate of left racetrack boundary
t_l_y=t_l(1:1:end,2); % y coordinate of left racetrack boundary
middlec1x =  (c1_r(1:1:end,1) + c1_l(1:1:end,1))/2;
middlec2x =  (c2_r(1:1:end,1) + c2_l(1:1:end,1))/2;
middlec1y =  (c1_r(1:1:end,2) + c1_l(1:1:end,2))/2;
middlec2y =  (c2_r(1:1:end,2) + c2_l(1:1:end,2))/2;
% middle_x = [line_2' x_2']%;middlec2x middlec2y;line_2' x_1'];
middle_x_ = [middlec1x middlec1y;x_1' line_1';middlec2x middlec2y ;x_2' line_2' ];
middle_x = middle_x_(1:1:end,1);
middle_y = middle_x_(1:1:end,2);
for i =1:1:size(states_history,1)
    x = states_history(i,:);

J =((x-x_desired)*Q*(x-x_desired)')^2;
% if x(3) < 0.7
%     J = J + 100000000000;
% end

kk = dsearchn([middle_x middle_y],[x(end,1) x(end,2)]);

if kk-1 < 1
    kk = 2;
end
if kk+1 > 600
    kk = 600-1;
end

x1=middle_x(kk+1);
x2=middle_x(kk-1);
y1=middle_y(kk+1);
y2=middle_y(kk-1);
distance_to_line = abs((y2-y1)*x(end,1) -...
    (x2-x1)*x(end,2) + x2*y1...
    -y2*x1)...
    /(sqrt((y2-y1)^2 +(x2-x1)^2));


strafe = 0;
if distance_to_line>5
    strafe = 1000000000;
end

Cost(i) = strafe+ J+ 1000*distance_to_line^2;%  + 1*gamma*u(t-1,:)*inv(diag(ones(1, size(u,2))).*sigma)*v(t-1,:)';% +intrsct_l*100000000 + intrsct_r*100000000;
distance(i) = distance_to_line;
end
figure
t = linspace(0,size(Cost,2),size(Cost,2))/10;
plot(t, Cost(:)')

axis tight
title(['Running Cost'],'Interpreter','latex');

xlabel('$t$[s]','Interpreter','latex');
ylabel('Running cost','Interpreter','latex');
legend({'Running cost'})
drawnow()
print('-depsc', 'single_track_infinity_runningcost_sim1_narrow.eps')
print('-dpng', 'single_track_infinity_runningcost_sim1_narrow.png')
% 
% % Error plots
% 
%%
figure
t = linspace(0,size(Cost,2),size(Cost,2))/10;
plot(t, sqrt(distance(:).^2)')

% title(['Distance to trajectory'],'Interpreter','latex');
curve1_x = [11 11];
curve1_y = [0 max(sqrt(distance(:).^2))];
line(curve1_x,curve1_y, 'Color','red')
curve1_x_end = [17.4 17.4];
curve1_y_end = [0 max(sqrt(distance(:).^2))];
line(curve1_x_end,curve1_y_end, 'Color','red')
curve2_x = [26.4 26.4];
curve2_y = [0 max(sqrt(distance(:).^2))];
line(curve2_x,curve2_y, 'Color','red')
curve2_x_end = [32.8 32.8];
curve2_y_end = [0 max(sqrt(distance(:).^2))];
line(curve2_x_end,curve2_y_end, 'Color','red')
txt1 = 'curve 1';
txt1_x = 11.1;
txt1_y = 1.3;
txt2 = 'curve 2';
txt2_x = 26.5;
txt2_y = 1.3;
% 
% % itm_formatfig('LatexNarrow')
% 
text(txt1_x,txt1_y,txt1,'Color','red','FontSize',6)
text(txt2_x,txt2_y,txt2,'Color','red','FontSize',6)

xlabel('t [s]');
ylabel('distance [m]');
% axis tight
itm_formatfig('LatexNarrow')
% legend({'distance to trajectory'})
drawnow()
print('-depsc', 'single_track_infinity_error_sim1_narrow.eps')
print('-dpng', 'single_track_infinity_error_sim1_narrow.png')

% mean(states_history(:,3))

