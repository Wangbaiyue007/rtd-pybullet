clear; clc;
%% load data
load('test_for_baiyue.mat');

%% save joint trajectories
joint_pos = summary.trajectory([1:2:13], :);
joint_vel = summary.trajectory([2:2:14], :);
writematrix(joint_pos, 'joint_pos.csv');
writematrix(joint_vel, 'joint_vel.csv');

%% save obstacles
numObstacles = size(summary.obstacles, 2);
Z = [];
for obs = 1:numObstacles
    z = summary.obstacles{1,obs}.Z';
    Z = [Z; z];
end
writematrix(Z, 'matlab_obstacles.csv');

%% save FO
for iter = 1:26
    for link = 1:9
        vertices_slc = [];
        vertices = [];
        for timestep = 1:100
            % merge all timestep
            ver_slc = P.info.sliced_FO_zono{1, iter}{1, timestep}{1, link}.Z';
            ver = P.info.FO_zono{1, iter}{1, timestep}{1, link}.Z';
            vertices_slc = [vertices_slc; ver_slc];
            vertices = [vertices; ver];
        end
        writematrix(vertices, append('matlab_sim/matlab_vertices_step', ...
                     num2str(iter), '_link', num2str(link), '.csv'));
        writematrix(vertices_slc, append('matlab_sim/matlab_vertices_slc_step', ...
                     num2str(iter), '_link', num2str(link), '.csv'));
    end
end