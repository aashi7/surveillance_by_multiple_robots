function [plan] = demo()

map = zeros(8,8);

[plan] = mex_planner(map);