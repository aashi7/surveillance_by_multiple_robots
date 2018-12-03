function [plan] = demo()

map = zeros(8,8);

%%%% add obstacles %%%%

wayPts = [[4,8,1];[8,8,3]]; %% row-Positions of all waypoints in first row, col-Positions of all waypoints in second row
starts = [[1,7];[1,2]]; %% row-Positions of all starts in first row, col-Positions of all waypoints in second row 
goals = [[1,5];[1,5]]; 

[plan] = mex_planner(map, wayPts, starts, goals);
[NumRobots, MaxPathLength, numofDOFs] = size(plan);

figure
imagesc(map);
%%%% Mark waypoints and robot starts on map %%%%
hold on
% h = zeros(2,1);
% h(1) = plot(starts(1,1),starts(1,2),'o');
% h(2) = plot(wayPts(1,1),wayPts(1,2),'h');
for i=1:size(starts,2)
    plot(starts(2,i), starts(1,i), 'o','MarkerFaceColor',[0,0,1] ,'MarkerSize', 15);
end

for i=1:size(goals,2)
    plot(goals(2,i), goals(1,i), 'o','MarkerFaceColor',[0,1,0] ,'MarkerSize', 15);
end

for w=1:size(wayPts,2)
    plot(wayPts(2,w), wayPts(1,w), 'h', 'MarkerFaceColor', [1,0,0], 'MarkerSize', 30);
end
%legend(h,'Robot Start', 'Waypoint');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
cols = jet;
for r = 1:NumRobots
    path_robot = squeeze(plan(r,:,:));
    %% Plot path_robot on map 
    %% assign color to the path 
    col_idx = cols(r,:);
    for i=1:MaxPathLength 
        %% Map grid coordinates to (x,y)
        plot(path_robot(i,2)+1, path_robot(i,1)+1, '-o', 'LineWidth', 10, 'Color', col_idx);
        %plot(path_robot(i,1)+1, path_robot(i,2)+1, 'LineWidth', 20 ,'Color', col_idx);
    end
end