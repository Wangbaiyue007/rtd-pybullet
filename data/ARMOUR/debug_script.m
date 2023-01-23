close all; clear; clc;

%% read robot configuration
robot = importrobot("../../assets/kinova_gen3_7dof/kinova_with_gripper_dumbbell.urdf");
configuration = [0.28159142, 0.6, 3.6350845 , 4.73481646, 2.55565072, 0.83405794, 2.05711487];
conf = homeConfiguration(robot);
for i = 1:7
    conf(i).JointPosition = configuration(i);
end

%% verification
figure(1); view(3); axis equal; hold on; axis on;
tspan = linspace(0, 1, 128 + 1);

num_steps = 54;
for i = 1:num_steps
    link_reachset_center{i} = readmatrix(append('reachsets/step_', num2str(i),'_center.txt'), 'FileType', 'text');
    link_reachset_generators{i} = readmatrix(append('reachsets/step_', num2str(i), '_radius.txt'), 'FileType', 'text');
end

% torque_reachset_center = readmatrix('reachsets/armour_constraints.out', 'FileType', 'text');
% torque_reachset_radius = readmatrix('reachsets/armour_control_input_radius.out', 'FileType', 'text');

% for tid = 1:128
tid = 128;

% choose a random time inside this time interval
t_lb = tspan(tid);
t_ub = tspan(tid + 1);
t = (t_ub - t_lb) * rand + t_lb;

% plot robot
show(robot, conf, "Frames", "off");

% plot obstacles
cube_pos = [0.43014; 0.16598; 0.172];
cube_size = [0.16; 0.16; 0.16];
rack_pos = [[-0.3, 0., 0.5]', [0.8, 0, 1]', [1, -0.6, 0]', [1, 0.81, 0]'];
rack_size = 0.5.*[[0.01, 2, 2]', [2, 2, 0.01]', [2, 0.01, 2]', [2, 0.01, 2]'];
obs_pos = [cube_pos rack_pos];
obs_size = [cube_size rack_size];
obs_ori = [[0, 0, 0.362*2]', zeros(3,4)];
for i = 1:size(obs_pos,2)
    c = obs_pos(:, i);
    g = eul2rotm(obs_ori(:, i)', "XYZ")*diag(obs_size(:,i));
    z = zonotope(c, g);
    v = vertices(z)';
    trisurf(convhulln(v),v(:,1),v(:,2),v(:,3),'FaceColor',[1,0,0],'FaceAlpha',0.1,'EdgeColor',[1,0,0],'EdgeAlpha',0.1);
end

% plot link reachsets
for step = 52
    for j = 1:8
        c = link_reachset_center{step}((tid-1)*8+j, :)';
        g = link_reachset_generators{step}( ((tid-1)*8+j-1)*3+1 : ((tid-1)*8+j)*3, :);
        Z = zonotope(c, g);
        Z_v = vertices(Z)';
        trisurf(convhulln(Z_v),Z_v(:,1),Z_v(:,2),Z_v(:,3),'FaceColor',[0,0,1],'FaceAlpha',0.1,'EdgeColor',[0,0,1],'EdgeAlpha',0.1);
    end
end

