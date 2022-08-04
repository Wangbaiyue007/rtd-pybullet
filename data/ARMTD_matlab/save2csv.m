clear; clc;
%% load data
load('test_for_baiyue.mat');

%% save joint trajectories
writematrix('joint_traj.csv', summary.trajectory);

%% save 