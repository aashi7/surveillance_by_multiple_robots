function [plan] = demo()

map = zeros(8,8);

wayPts = [[4,8];[8,8]];
starts = [[1,1];[1,1]];
goals = [[1,1];[1,1]];

[plan] = mex_planner(map);
[NumRobots, MaxPathLength, numofDOFs] = size(plan);

figure
imagesc(map);
%%%% Mark waypoints and robot starts on map %%%%
hold on
% h = zeros(2,1);
% h(1) = plot(starts(1,1),starts(1,2),'o');
% h(2) = plot(wayPts(1,1),wayPts(1,2),'h');
for i=1:size(starts,1)
    plot(starts(i,1), starts(i,2), 'o','MarkerFaceColor',[0,0,1] ,'MarkerSize', 15);
end

for w=1:size(wayPts,1)
    plot(wayPts(w,1), wayPts(w,2), 'h', 'MarkerFaceColor', [1,0,0], 'MarkerSize', 20);
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
        plot(path_robot(i,1)+1, path_robot(i,2)+1, '-o', 'LineWidth', 10, 'Color', col_idx);
        %plot(path_robot(i,1)+1, path_robot(i,2)+1, 'LineWidth', 20 ,'Color', col_idx);
    end
end