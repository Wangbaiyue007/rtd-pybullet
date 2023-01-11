%% Testing differences between the original urdf and the approximated urdf
% 
clear; clc; close all;

%% import robot
robot_original = importrobot('kinova_with_gripper_dumbbell.urdf');
robot_original.Gravity = [0 0 -9.8];
robot_original.DataFormat = 'column';
robot_approx = importrobot('kinova_with_gripper_dumbbell_approx.urdf');
robot_approx.Gravity = [0 0 -9.8];
robot_approx.DataFormat = 'column';

%% testing
q = randomConfiguration(robot_original);
qd = 0.5 - rand(7,1);
tau = 0.5 - rand(7,1);
figure(1);
show(robot_original, q);
figure(2);
show(robot_approx, q);

qdd_original = forwardDynamics(robot_original, q, qd, tau);
qdd_approx = forwardDynamics(robot_approx, q, qd, tau);
error = norm(qdd_original - qdd_approx)/norm(qdd_original);
disp(error)