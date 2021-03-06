function [plans] = demo()
close all; clc; clear;

% map = ones(8,8); C = 2;
% wayPts = [[4,8,2,1,4];[8,4,3,5,7]];
% starts = [[1,8];[1,8]];
% goals = [[8,1];[8,1]];

[~,C,~,T,map] = readproblem('maps/map3.txt');

% tsz = size(T,1);
% wayPts = ([T(tsz,:); T(ceil(tsz-(tsz/3)),:); T(ceil(tsz/3),:)])';
% starts = ([T(ceil(tsz/4),:); T(ceil(tsz-(tsz/4)),:)])';
% goals = ([T(ceil(tsz/2),:); T(ceil(tsz/2),:)])';

% starts = [28, 50, 192; 24, 125, 344];
starts = [28, 84, 192; 24, 240, 344];
goals = [430, 430, 430; 186, 186, 186];
wayPts = [265, 370, 378, 85, 276, 50, 110; 70, 356, 24, 60, 234, 190, 290];
% wayPts = [265, 370, 378, 85, 276; 70, 356, 24, 60, 234];
% wayPts = [238, 225, 371; 328, 297, 300];

numRobots = size(starts,2);

video = VideoWriter('videos/test1.avi');
open(video);

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
    plot(goals(1,i), goals(2,i), 'o','MarkerFaceColor',[0,0,0.5] ,'MarkerSize', 15);
end

for w=1:size(wayPts,2)
    plot(wayPts(1,w), wayPts(2,w), 'h', 'MarkerFaceColor', [1,0,0], 'MarkerSize', 30);
end

[plans, ~] = planner(map, C, starts, goals, wayPts);
max_planlength = size(plans,2);

cols = ['g','g','g']; widths = [2, 2, 2];
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
    frame = getframe(gcf);
    writeVideo(video, frame);
end