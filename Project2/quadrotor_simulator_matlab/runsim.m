close all;
clear all;
clc;
addpath(genpath('./'));

%% Plan path
disp('Planning ...');
map = load_map('maps/map0.txt', 0.1, 1.0, 0.25);

% Circle Trajectory start ans stop 
% start = {[1.0 0.0 0.0]};
% stop = {[1.0 0.0 0.5]};

% Diamond Trajectory start ans stop 
start = {[0.0 0.0 0.0]};
stop = {[1.0 0.0 0.0]};

nquad = length(start);
path = {[0,0,0]};

%% Additional init script
init_script;

%% Run trajectory
trajectory = test_trajectory(start, stop, map, path, true); % with visualization
