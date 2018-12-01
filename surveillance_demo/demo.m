%%%%% Surveillance Demo %%%%%
%%%%% Call the cpp function from here %%%%

%% [output] = planner(inputs)

function [plan] = demo()

%% 8 x 8 grid 
map = zeros(8,8);

starts = [1 1; 8 8];
goals = [8 8; 1 1];
waypoints = [3 7; 4 2];

[plan] = mex_planner(map, starts, goal, waypoints);

%% For visualization - imagesc - id for waypoint, robot and obstacle 