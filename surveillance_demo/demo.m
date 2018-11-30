%%%%% Surveillance Demo %%%%%
%%%%% Call the cpp function from here %%%%

%% [output] = planner(inputs)

function [plan] = demo()

%% 8 x 8 grid 
map = zeros(8,8);

[plan] = mex_planner(map);