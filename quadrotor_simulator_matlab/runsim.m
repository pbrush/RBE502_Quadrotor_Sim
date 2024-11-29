close all;
clear all;
clc;
addpath(genpath('./'));

%% Plan path
disp('Planning ...');
map = load_map('maps/map0.txt', 0.1, 1.0, 0.25);

% start = {[2.0  3 5.0]}; %map3
% stop  = {[18.0  4.0 5.0]};
% start = {[0.1 5.0 1.0]}; %map2
% stop  = {[5.0 10.0 3.0]};
% start = {[2.0  -2.0 1.0]};%map1
% stop  = {[8.0 18.0 2.5]};
start = {[2.0 2.0 2.0]}; %map0
stop  = {[0.0 0.0 0.0]};
nquad = length(start);

%% Additional init script
init_script;

%% Run trajectory
trajectory = test_trajectory(start, stop, map, path, true); % with visualization
