clear; clc;
%% load data
load('table_demo_bernstein_pi36.mat');

%% save joint trajectories
joint_pos = summary.trajectory([1:2:13], :);
joint_vel = summary.trajectory([2:2:14], :);
joint_acc = A.reference_acceleration;
writematrix(joint_pos, 'joint_pos.csv', 'Delimiter', ',');
writematrix(joint_vel, 'joint_vel.csv', 'Delimiter', ',');
writematrix(joint_acc, 'joint_acc.csv', 'Delimiter', ',');

%% save obstacles
numObstacles = size(summary.obstacles, 2);
Z = [];
for obs = 1:numObstacles
    z = summary.obstacles{1,obs}.Z';
    Z = [Z; z];
end
writematrix(Z, 'matlab_obstacles.csv');

%% save FO
clearvars -except P;
for iter = 1:12
    for link = 1:9
        verts_slc_ = [];
        verts_ = [];
        for timestep = 1:100
            % merge all timestep, reduce zonotope
            zono_slc = P.info.sliced_FO_zono{1, iter}{1, timestep}{1, link};
            zono = P.info.FO_zono{1, iter}{1, timestep}{1, link};
            % reduce generators
            zono_slc_ = reduce(zono_slc, 'pca', 4);
            zono_ = reduce(zono, 'pca', 4);
            % convert to vertices
            ver_slc_ = vertices(zono_slc_);
            ver_ = vertices(zono_);
            verts_slc_ = [verts_slc_ ver_slc_];
            verts_ = [verts_ ver_];
        end
        %writematrix(verts_, append('matlab_sim/matlab_vertices_step', ...
        %             num2str(iter), '_link', num2str(link), '.csv'));
        %writematrix(verts_slc_, append('matlab_sim/matlab_vertices_slc_step', ...
        %             num2str(iter), '_link', num2str(link), '.csv'));
        % to see the convex hull
        % [K, v] = convhull(verts_(1,:), verts_(2,:), verts_(3,:));
        % trisurf(K,verts_(1,:), verts_(2,:), verts_(3,:), 'FaceColor', 'cyan', 'FaceAlpha', 0.5);
        % clean workspace
        %% write to stl
        % generate convex hull for each link at each planning step
        [K,v] = convhull(verts_(1,:),verts_(2,:),verts_(3,:));
        [K_slc,v_slc] = convhull(verts_slc_(1,:),verts_slc_(2,:),verts_slc_(3,:));
        stlwrite(append('../../assets/zonotope/meshes_matlab/matlab_mesh_step', num2str(iter), '_link', num2str(link), '.stl'), K, verts_');
        stlwrite(append('../../assets/zonotope/meshes_matlab/matlab_mesh_slc_step', num2str(iter), '_link', num2str(link), '.stl'), K_slc, verts_slc_');
        clear verts_ verts_slc_;
    end
end