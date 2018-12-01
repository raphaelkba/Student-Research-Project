% Creates a squared grid map for the simulation of the single track model
% (Robotics toolbox required)
% Inputs: None
% Output: map: the designed grid map with n by m size
%         cost_map: matrix n by m, with a cost for each square in the grid map

function [map, cost_map, obstacles] = create_map()

% Set the size of the map in meters
size_grid_x = 100; % n
size_grid_y = 100; % m
% Size of each square on the grid in meters
size_quare = 1;

% Creates the map with help of the robotic toolbox
map = robotics.BinaryOccupancyGrid(size_grid_x,size_grid_y,size_quare);

% Create the obstacles(walls) in the grid map
obstacles = [];
for x=0:1:size_grid_x    
    for y=0:1:size_grid_y
        
        if x==0 || y == 0
           obstacles = [obstacles;x y];  
        end

        if x==size_grid_x || y == size_grid_y
           obstacles = [obstacles;x y];  
        end
        
        if x > 12 && x<=88 && y > 12 && y <=88
           obstacles = [obstacles;x y];  
        end
   
    end
end
setOccupancy(map,obstacles,1);

% Shows current map
% figure
% show(map)

% Set a grid for the map
% grid on
% set(gca,'XTick',0:2:size_grid_x,'YTick',0:2:size_grid_y)   



% Initialzes cost map
cost_map = zeros(size_grid_x,size_grid_y);

% Creates the cost for each square of the grid
for x=1:1:size_grid_x    
    for y=1:1:size_grid_y
                
        if y == 12
           cost_map(x,y) =10000; 
        end
        
        if x == 12
           cost_map(x,y) =10000; 
        end
            
        if y == 11
           cost_map(x,y) =1000; 
        end
        
        if x == 11
           cost_map(x,y) =1000; 
        end
        
        
        if y == 10
           cost_map(x,y) =100; 
        end
        
        if x == 10
           cost_map(x,y) =100; 
        end
        
        if x == 89
           cost_map(x,y) =10000; 
        end
        
        if y == 89
           cost_map(x,y) =10000; 
        end
        
        if x == 90
           cost_map(x,y) =100; 
        end
        
        if y == 90
           cost_map(x,y) =100; 
        end
        
        if x == 91
           cost_map(x,y) =100; 
        end
        
        if y == 91
           cost_map(x,y) =100; 
        end
        
              
        if x == 9
           cost_map(x,y) =10; 
        end
        
        if y == 9
           cost_map(x,y) =10; 
        end
        
        if x == 92
           cost_map(x,y) =10; 
        end
        
        if y == 92
           cost_map(x,y) =10; 
        end
        
        if y == 8
           cost_map(x,y) =5; 
        end
        
        if x == 8
           cost_map(x,y) =5; 
        end
        
        if y == 93
           cost_map(x,y) =5; 
        end
        
        if x == 93
           cost_map(x,y) =5; 
        end
        
        if y == 7
           cost_map(x,y) =0; 
        end
        
        if x == 7
           cost_map(x,y) =0; 
        end
        
        if y == 94
           cost_map(x,y) =0; 
        end
        
        if x == 94
           cost_map(x,y) =0; 
        end        
        
        if y == 5
           cost_map(x,y) =10; 
        end
        
        if x == 5
           cost_map(x,y) =10; 
        end
        
        if y == 96
           cost_map(x,y) =10; 
        end
        
        if x == 96
           cost_map(x,y) =10; 
        end
        
        if y == 4
           cost_map(x,y) =100; 
        end
        
        if x == 4
           cost_map(x,y) =100; 
        end
        
        if y == 97
           cost_map(x,y) =100; 
        end
        
        if x == 97
           cost_map(x,y) =100; 
        end
        
         if y == 3
           cost_map(x,y) =1000; 
        end
        
        if x == 3
           cost_map(x,y) =1000; 
        end
        
        if y == 98
           cost_map(x,y) =1000; 
        end
        
        if x == 98
           cost_map(x,y) =1000; 
        end
        
        
        
        
        if y == 2
           cost_map(x,y) =10000; 
        end
        if x == 2
           cost_map(x,y) =10000; 
        end
        if x == 99
           cost_map(x,y) =10000; 
        end
        if y == 99
           cost_map(x,y) =10000; 
        end
        
        
        if x == 6 && y <= 95 && y >= 6        
            cost_map(x,y) = 5;            
        end
        if x == 95 && y <= 95 && y >= 6        
            cost_map(x,y) = 5;            
        end
        
        if y == 6 && x < 96 && x > 6        
            cost_map(x,y) = 5;            
        end
        
        if y == 95 && x < 96 && x > 5        
            cost_map(x,y) = 5;            
        end
        
        
    end
    
end

% Handly shaped cost for the corners of the grid
%left top corner
cost_map(8,8) = 0; 
cost_map(9,9) = 0;
cost_map(10,10) = 0;
cost_map(10,10) = 0;
cost_map(9,8) = 0;
cost_map(10,8) = 0;
cost_map(10,9) = 0;
cost_map(11,9) = 0;
cost_map(11,8) = 0;
cost_map(8,9) = 0;
cost_map(8,10) = 0;
cost_map(8,11) = 0;
cost_map(8,12) = 0;
cost_map(8,13) = 0;
cost_map(9,10) = 0;
cost_map(9,11) = 0;
cost_map(9,12) = 0;
cost_map(12,9) = 0;
cost_map(12,8) = 0;
cost_map(13,9) = 0;
cost_map(13,8) = 0;
cost_map(8,14) = 0;
cost_map(8,15) = 0;
cost_map(8,16) = 0;
cost_map(9,13) = 0;
cost_map(9,14) = 0;
cost_map(7,7) = 5;
cost_map(6,6) = 5;
cost_map(7,8) = 5;
cost_map(8,7) = 5;
cost_map(9,14) = 0;
cost_map(12,8) = 0;
cost_map(13,8) = 0; 
cost_map(14,8) = 0; 
cost_map(15,8) = 0;
cost_map(16,8) = 0;
cost_map(12,9) = 0; 
cost_map(13,9) = 0; 
cost_map(14,9) = 0; 

%right bottom corner
cost_map(94,94) = 0; 
cost_map(93,93) = 0;
cost_map(92,92) = 0;
cost_map(93,94) = 0;
cost_map(92,94) = 0;
cost_map(92,93) = 0;
cost_map(91,93) = 0;
cost_map(91,94) = 0;
cost_map(94,93) = 0;
cost_map(94,92) = 0;
cost_map(94,91) = 0;
cost_map(94,90) = 0;
cost_map(94,89) = 0;
cost_map(93,92) = 0;
cost_map(93,91) = 0;
cost_map(93,90) = 0;
cost_map(90,93) = 0;
cost_map(90,94) = 0;
cost_map(89,93) = 0;
cost_map(89,94) = 0;
cost_map(94,88) = 0;
cost_map(94,87) = 0;
cost_map(94,86) = 0;
cost_map(93,89) = 0;
cost_map(93,88) = 0;
cost_map(94,94) = 5;
cost_map(96,96) = 5;
cost_map(93,94) = 0;
cost_map(94,93) = 0;
cost_map(93,88) = 0;
cost_map(90,94) = 0;
cost_map(89,94) = 0; 
cost_map(88,94) = 0; 
cost_map(87,94) = 0;
cost_map(86,94) = 0;
cost_map(90,93) = 0; 
cost_map(89,93) = 0; 
cost_map(88,93) = 0; 


%left bot corner

cost_map(94,8) = 0; 
cost_map(93,9) = 0;
cost_map(92,10) = 0;
cost_map(92,10) = 0;
cost_map(93,8) = 0;
cost_map(92,8) = 0;
cost_map(92,9) = 0;
cost_map(91,9) = 0;
cost_map(91,8) = 0;
cost_map(94,9) = 0;
cost_map(94,10) = 0;
cost_map(94,11) = 0;
cost_map(94,12) = 0;
cost_map(94,13) = 0;
cost_map(93,10) = 0;
cost_map(93,11) = 0;
cost_map(93,12) = 0;
cost_map(90,9) = 0;
cost_map(90,8) = 0;
cost_map(89,9) = 0;
cost_map(89,8) = 0;
cost_map(94,14) = 0;
cost_map(94,15) = 0;
cost_map(94,16) = 0;
cost_map(93,13) = 0;
cost_map(93,14) = 0;
cost_map(95,7) = 5;
cost_map(96,6) = 5;
cost_map(95,8) = 5;
cost_map(94,7) = 5;
cost_map(93,14) = 0;
cost_map(90,8) = 0;
cost_map(89,8) = 0; 
cost_map(88,8) = 0; 
cost_map(87,8) = 0;
cost_map(86,8) = 0;
cost_map(90,9) = 0; 
cost_map(89,9) = 0; 
cost_map(88,9) = 0; 
cost_map(91,92) = 0; 
cost_map(90,92) = 5;
cost_map(90,93) = 0;




%right top corner

cost_map(8,94) = 0; 
cost_map(9,93) = 0;
cost_map(10,92) = 0;
cost_map(10,92) = 0;
cost_map(9,94) = 0;
cost_map(10,94) = 0;
cost_map(10,93) = 0;
cost_map(11,93) = 0;
cost_map(11,94) = 0;
cost_map(8,93) = 0;
cost_map(8,92) = 0;
cost_map(8,91) = 0;
cost_map(8,90) = 0;
cost_map(8,89) = 0;
cost_map(9,92) = 0;
cost_map(9,91) = 0;
cost_map(9,90) = 0;
cost_map(12,93) = 0;
cost_map(12,94) = 0;
cost_map(13,93) = 0;
cost_map(13,94) = 0;
cost_map(8,88) = 0;
cost_map(8,87) = 0;
cost_map(8,86) = 0;
cost_map(9,89) = 0;
cost_map(9,88) = 0;
cost_map(7,95) = 5;
cost_map(6,96) = 5;
cost_map(7,94) = 5;
cost_map(8,95) = 5;
cost_map(9,88) = 0;
cost_map(12,94) = 0;
cost_map(13,94) = 0; 
cost_map(14,94) = 0; 
cost_map(15,94) = 0;
cost_map(16,94) = 0;
cost_map(12,93) = 0; 
cost_map(13,93) = 0; 
cost_map(14,93) = 0; 
cost_map(9,87) = 0; 
cost_map(8,85) = 0; 
cost_map(10,91) = 0; 
cost_map(11,92) = 0; 
cost_map(12,91) = 0; 
cost_map(13,91) = 0; 

%dont know
cost_map(91,10) = 0; 
cost_map(92,11) = 0; 
cost_map(91,12) = 0; 
cost_map(91,13) = 0; 

% obstacle = [];
% obstacle = [obstacle;3 50;4 50; 5 50;2 50;1 50;3 51;4 51; 5 51;2 51;1 51;...
% 6 50;6 51; 7 50; 7 51; 3 52;4 52; 5 52;2 52;1 52;7 52;];
% setOccupancy(map,obstacle,1);
% obstacle = [obstacle;12 70;11 70;10 70;9 70;12 71;11 71;10 71;9 71;...
% 12 72;11 72;10 72;9 72];
% setOccupancy(map,obstacle,1);


% itm_formatfig('LatexWide')
% % Print the cost in each square of the grid
% for i=1:1:size(cost_map,1)-80
%     for j=1:1:size(cost_map,2)-80
%         if getOccupancy(map,[i,j])
%             xy = grid2world(map,[i j]);
%             txt1 =  num2str(0);
%             text(xy(1)-0.5,xy(2),txt1,'FontSize',6,'Color','red')
%         else
%         xy = grid2world(map,[i j]);
%         txt1 = num2str(cost_map(i,j));
%         text(xy(1)-0.5,xy(2),txt1,'FontSize',6,'Color','red')
%         end
%     end
% end
% % title(['grid Map Costs'],'Interpreter','latex');
% xlabel('x[m]');
% ylabel('y[m]');
% 
% % print('-depsc', 'gridmap_cost_.eps')
% % print('-dpng', 'gridmap_cost_.png')
% yo = 1;