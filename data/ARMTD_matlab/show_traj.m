clear; clc; close all;
load('test0822.mat');
%%
robot = A.robot;
joint_pos = summary.trajectory([1:2:13], :);
config = zeros(7,1);

for i = 1:5:size(joint_pos, 2)
    robot.show(joint_pos(:, i), 'PreservePlot', 0);
    drawnow
end