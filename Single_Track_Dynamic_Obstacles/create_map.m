% Creates a squared grid map for the simulation of the single track model
% (Robotics toolbox required)
% Inputs: None
% Output: map: the designed grid map with n by m size
%         cost_map: matrix n by m, with a cost for each square in the grid map

function [map, cost_map, obstacles] = create_map()

% Set the size of the map in meters
size_grid_x = 150; % 
size_grid_y = 13; % m
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
   
    end
end
setOccupancy(map,obstacles,1);

% Shows current map
% show(map)

% Set a grid for the map
% grid on
% set(gca,'XTick',0:1:size_grid_x,'YTick',0:1:size_grid_y)   



% Initialzes cost map
cost_map = zeros(size_grid_y,size_grid_x);

% Creates the cost for each square of the grid
for x=1:1:size_grid_y    
    for y=1:1:size_grid_x
                        
        if x == 2
           cost_map(x,y) =5; 
        end
        
        if x == 3
           cost_map(x,y) =4; 
        end
        if x == 4
           cost_map(x,y) =3; 
        end
        if x == 5
           cost_map(x,y) =2; 
        end
        if x == 6
           cost_map(x,y) =1; 
        end
        if x == 7
           cost_map(x,y) =0; 
        end
        if x == 8
           cost_map(x,y) =1; 
        end
        if x == 9
           cost_map(x,y) =2; 
        end
        if x == 10
           cost_map(x,y) =3; 
        end
        if x == 11
           cost_map(x,y) =4; 
        end
        if x == 12
           cost_map(x,y) =5; 
        end
                
    end
    
end



% show(map)
% 
% % Print the cost in each square of the grid
% for i=1:1:size(cost_map,1)
%     for j=1:1:size(cost_map,2)
%         xy = grid2world(map,[i j]);
%         txt1 = num2str(cost_map(i,j));
%         text(xy(1)-0.5,xy(2),txt1,'FontSize',5,'Color','red')
%     end
% end
