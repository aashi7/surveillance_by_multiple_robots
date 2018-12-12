function [plans] = demo()

close all;clear;

%     load('map3.mat');
%     starts = [28, 84, 192; 24, 240, 344];
%     goals = [430, 430, 430; 186, 186, 186];
%     wayPts = [238, 225, 371; 328, 297, 300];
%wayPts = [265, 370, 378, 85, 276; 70, 356, 24, 60, 234];

% map = zeros(8,8);
% map(4,4) = 1;
% map(3,3) = 1;
% map(2,1)  = 1; %% col first and then row 
%     
% wayPts = [[8,8,3];[4,8,1]];
% starts = [[1,2,1];[1,7,1]];
% goals = [[1,5,1];[1,5,1];]; 

[~, C, ~, T, map] = readproblem('map3.txt');
% map = map - 1;
% map = map./max(map(:));
% map = round(map);
%starts = [28, 50, 192, 110; 24, 125, 344, 290];
%goals = [430, 430, 430, 430; 186, 186, 186, 186]; 
starts = [28, 50, 192; 24, 125, 344]; %% 3 Robots
% % starts = [28, 84, 192; 24, 240, 344];
goals = [430, 430, 430; 186, 186, 186]; %% 3 Goals
%starts = [28, 50; 24, 125];
%goals = [430, 430; 186, 186];
%wayPts = [265, 370, 378, 85, 276, 50, 110, 225; 70, 356, 24, 60, 234, 190, 290, 297]; %% 7W
%wayPts = [265, 370, 378, 85, 276, 50; 70, 356, 24, 60, 234, 190]; %%% For Case 4: 3R, 6W
wayPts = [265, 370, 378, 85, 276; 70, 356, 24, 60, 234]; %%%% For Case 3: 3R, 5W 
%wayPts = [238, 225, 371; 328, 297, 300];  %%%%% For Case 1: 3R, 3W
%wayPts = [265, 370, 378, 85; 70, 356, 24, 60]; %%% For Case 2: 3R, 4W 
%wayPts = [265, 85; 70, 60];
%wayPts = [85; 60];
numRobots = size(starts,2);

%video = VideoWriter('videos/test2.avi');
%open(video);

figure('units','normalized','outerposition',[0 0 1 1]);
mycolMap = jet(256);
mycolMap(1,:) = 1;
imagesc(map'); axis square; colorbar; colormap(mycolMap); hold on;
% figure
% imagesc(map);
%%%% Mark waypoints and robot starts on map %%%%
hold on
for i=1:size(starts,2)
    plot(starts(1,i), starts(2,i), 'o','MarkerFaceColor',[0,0,1] ,'MarkerSize', 15);
    plot(goals(1,i), goals(2,i), 's','MarkerFaceColor',[0,0,0.5] ,'MarkerSize', 15);
end

for w=1:size(wayPts,2)
    plot(wayPts(1,w), wayPts(2,w), 'h', 'MarkerFaceColor', [1,0,0], 'MarkerSize', 30);
end

plans = planner(map, C, wayPts, starts, goals);
max_planlength = size(plans,2);

cols = ['b','g','k']; widths = [2, 2, 2];
for t = 1:max_planlength
    for r = 1:numRobots
        path_robot = squeeze(plans(r,:,:));
        %% Plot path_robot on map 
        %% assign color to the path 
        col = cols(r); width = widths(r);
        %% Map grid coordinates to (x,y)
        plot(path_robot(t,1)+1, path_robot(t,2)+1, '.', 'LineWidth', width, 'Color', col);
    end
    drawnow;
    %frame = getframe(gcf);
    %writeVideo(video, frame);
end

% video = VideoWriter('videos/demo1.avi');
% open(video);
% 
%     figure('units','normalized','outerposition',[0 0 1 1]);
%     myColorMap = jet(256);
%     myColorMap(1,:) = 1;
%     imagesc(map'); axis square; colorbar; colormap(myColorMap); hold on;
%     
% 
%     for i=1:size(starts,2)
%         plot(starts(1,i), starts(2,i), '>','MarkerFaceColor',[0,0,1] ,'MarkerSize', 15);
%         plot(goals(1,i), goals(2,i), 'd','MarkerFaceColor',[0,1,0] ,'MarkerSize', 20);
%     end
% 
%     for w=1:size(wayPts,2)
%         plot(wayPts(1,w), wayPts(2,w), 'h', 'MarkerFaceColor', [1,0,0], 'MarkerSize', 20);
%     end
%     
%     [plan] = planner(map, C, wayPts, starts, goals);
%     
%         %% If path found %%
%     max_planlength = size(plan,2);
%     if size(plan,2) > 0;
%         cols = jet;
%         for t=1:max_planlength
%         for r = 1:numRobots
%             path_robot = squeeze(plan(r,:,:));
%             %% assign color to the path 
%             col_idx = cols(r*10,:);
%             plot(path_robot(t,1)+1, path_robot(t,2)+1, '-', 'LineWidth', 1, 'Color', col_idx);
% 
%         end
%         drawnow;
%         frame = getframe(gcf);
%         writeVideo(video, frame);
%         end
%     end
    
