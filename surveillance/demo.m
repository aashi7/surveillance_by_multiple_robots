function [plans] = demo()
close all; clc; clear;

map = ones(8,8); C = 2;
wayPts = [[4,8];[8,4]];
starts = [[1,8];[1,8]];
goals = [[8,1];[8,1]];
numRobots = size(starts,2);

%[~,C,~,T,map] = readproblem('map3.txt');
% tsz = size(T,1);
% wayPts = ([T(tsz,:); T(ceil(tsz-(tsz/3)),:); T(ceil(tsz/3),:)])';
% starts = ([T(ceil(tsz/4),:); T(ceil(tsz-(tsz/4)),:)])';
% goals = ([T(ceil(tsz/2),:); T(ceil(tsz/2),:)])';

[plans, planLengths] = planner(map, C, starts, goals, wayPts);

max_planlength = 0;
for i=1:numRobots
    if(planLengths(i) > max_planlength)
        max_planlength = planLengths(i);
    end
end

figure('units','normalized','outerposition',[0 0 1 1]);
imagesc(map'); axis square; colorbar; colormap jet; hold on;
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

cols = ['b','k']; widths = [5, 2];
for r = 1:numRobots
    path_robot = squeeze(plans(r,:,:));
    %% Plot path_robot on map 
    %% assign color to the path 
    col = cols(r); width = widths(r);
    %% Map grid coordinates to (x,y)
    plot(path_robot(:,1)+1, path_robot(:,2)+1, '-o', 'LineWidth', width, 'Color', col);
end