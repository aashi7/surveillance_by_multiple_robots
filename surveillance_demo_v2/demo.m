function [plan] = demo()

map = zeros(8,8);

wayPts = [[4,8],[8,8]];
starts = [[0,0],[0,0]];
goals = [[0,0], [0,0]];

[plan] = mex_planner(map);
[NumRobots, MaxPathLength, numofDOFs] = size(plan);

figure
imagesc(map);
hold on 
for r = 1:NumRobots
    path_robot = squeeze(plan(r,:,:));
    %% Plot path_robot on map 
    for i=1:MaxPathLength 
        %% Map grid coordinates to (x,y)
    end
end