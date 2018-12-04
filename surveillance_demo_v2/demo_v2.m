
load('map3.mat');

starts = [28, 84, 192; 24, 240, 344]; %%
goals = [430, 430, 430; 186, 186, 186];
%wayPts = [265, 370, 378, 85, 276; 70, 356, 24, 60, 234];
wayPts = [238, 225, 371; 328, 297, 300];

figure('units','normalized','outerposition',[0 0 1 1]);
myColorMap = jet(256);
myColorMap(1,:) = 1;
imagesc(map'); axis square; colorbar; colormap(myColorMap); hold on;

hold on
for i=1:size(starts,2)
    plot(starts(1,i), starts(2,i), '>','MarkerFaceColor',[0,0,1] ,'MarkerSize', 15);
    plot(goals(1,i), goals(2,i), 'd','MarkerFaceColor',[0,1,0] ,'MarkerSize', 20);
end

for w=1:size(wayPts,2)
    plot(wayPts(1,w), wayPts(2,w), 'h', 'MarkerFaceColor', [1,0,0], 'MarkerSize', 20);
end

[plan] = mex_planner(map, wayPts, starts, goals);

%% If path found %%
if size(plan,3) > 0;
    cols = jet;
    for r = 1:NumRobots
        path_robot = squeeze(plan(r,:,:));
        %% Plot path_robot on map 
        %% assign color to the path 
        col_idx = cols(r,:);
        %for i=1:MaxPathLength 
            %% Map grid coordinates to (x,y)
            plot(path_robot(:,2)+1, path_robot(:,1)+1, '-o', 'LineWidth', 3, 'Color', col_idx);
            %plot(path_robot(i,1)+1, path_robot(i,2)+1, 'LineWidth', 20 ,'Color', col_idx);
        %end
    end
end