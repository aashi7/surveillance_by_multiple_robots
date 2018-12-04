function [plan] = demo()

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

starts = [28, 50, 192; 24, 125, 344];
% % starts = [28, 84, 192; 24, 240, 344];
goals = [430, 430, 430; 186, 186, 186];
%wayPts = [265, 370, 378, 85, 276, 50, 110; 70, 356, 24, 60, 234, 190, 290];
%wayPts = [265, 370, 378, 85, 276, 50; 70, 356, 24, 60, 234, 190];
%wayPts = [265, 370, 378, 85, 276; 70, 356, 24, 60, 234]; %%%% For Case 3: 3R, 3W 
%wayPts = [238, 225, 371; 328, 297, 300];  %%%%% For Case 1: 3R, 3W
wayPts = [265, 370, 378, 85; 70, 356, 24, 60]; %%% For Case 2: 3R, 4W 

numRobots = size(starts,2);

    figure('units','normalized','outerposition',[0 0 1 1]);
    myColorMap = jet(256);
    myColorMap(1,:) = 1;
    imagesc(map'); axis square; colorbar; colormap(myColorMap); hold on;
    

    for i=1:size(starts,2)
        plot(starts(1,i), starts(2,i), '>','MarkerFaceColor',[0,0,1] ,'MarkerSize', 15);
        plot(goals(1,i), goals(2,i), 'd','MarkerFaceColor',[0,1,0] ,'MarkerSize', 20);
    end

    for w=1:size(wayPts,2)
        plot(wayPts(1,w), wayPts(2,w), 'h', 'MarkerFaceColor', [1,0,0], 'MarkerSize', 20);
    end
    
    [plan] = planner(map, C, wayPts, starts, goals);
    
        %% If path found %%
    if size(plan,2) > 0;
        cols = jet;
        for r = 1:numRobots
            path_robot = squeeze(plan(r,:,:));
            %% assign color to the path 
            col_idx = cols(r,:);
            plot(path_robot(:,1)+1, path_robot(:,2)+1, '-', 'LineWidth', 1, 'Color', col_idx);

        end
    end
    
end