close all; clear; clc;

%% read robot configuration
robot = importrobot("../../assets/kinova_gen3_7dof/kinova_with_gripper_dumbbell.urdf");
configuration = [0.28159142, 0.6, 3.6350845 , 4.73481646, 2.55565072, 0.83405794, 2.05711487];
conf = homeConfiguration(robot);
for i = 1:7
    conf(i).JointPosition = configuration(i);
end

%% read CUDA output
link_reachset_center = readmatrix('reachsets/step_1_center.txt', 'FileType', 'text');
link_reachset_generators = readmatrix('reachsets/step_1_radius.txt', 'FileType', 'text');

% torque_reachset_center = readmatrix('reachsets/armour_constraints.out', 'FileType', 'text');
% torque_reachset_radius = readmatrix('reachsets/armour_control_input_radius.out', 'FileType', 'text');

%% verification
figure; view(3); axis equal; hold on; axis on;
show(robot, conf, "Frames", "off");
tspan = linspace(0, 1, 128 + 1);

% for tid = 1:128
tid = 128;

% choose a random time inside this time interval
t_lb = tspan(tid);
t_ub = tspan(tid + 1);
t = (t_ub - t_lb) * rand + t_lb;

% q = get_desired_traj(beta, t);

% plot robot
% A.plot_at_time(q);

% plot link reachsets
for j = 1:8
    c = link_reachset_center((tid-1)*8+j, :)';
    g = link_reachset_generators( ((tid-1)*8+j-1)*3+1 : ((tid-1)*8+j)*3, :);
    Z = zonotope(c, g);
    Z_v = vertices(Z)';
    trisurf(convhulln(Z_v),Z_v(:,1),Z_v(:,2),Z_v(:,3),'FaceColor',[0,0,1],'FaceAlpha',0.1,'EdgeColor',[0,0,1],'EdgeAlpha',0.3);
end

% end

% figure; hold on;
% 
% us = zeros(7,128);
% ts = zeros(1,128);
% for tid = 1:128
%     % choose a random time inside this time interval
%     t_lb = tspan(tid);
%     t_ub = tspan(tid + 1);
%     ts(tid) = (t_ub - t_lb) * rand + t_lb;
% 
%     [q, qd, qdd] = get_desired_traj(beta, ts(tid));
% 
%     us(:,tid) = rnea(q, qd, qd, qdd, true, params.nominal) + transmision_inertia' .* qdd;
% end
% 
% u_lb = torque_reachset_center - torque_reachset_radius;
% u_ub = torque_reachset_center + torque_reachset_radius;
% 
% % there is a better way to do this
% for i = 1:7
%     subplot(3,3,i);
%     hold on;
%     plot(ts, us(i,:), 'r');
%     plot(ts, u_lb(:,i), 'b');
%     plot(ts, u_ub(:,i), 'b');
%     title(['link ', num2str(i)]);
%     xlabel('time (sec)');
%     ylabel('torque (N*m)');
% end
% sgtitle('sliced torque reachable set');


%% helper functions
function [q, qd, qdd] = get_desired_traj(beta, t)
    [B, dB, ddB] = Bezier_kernel_deg5(t);
    
    q = zeros(7,1);
    qd = zeros(7,1);
    qdd = zeros(7,1);
    for j = 1:6
        q = q + beta{j} * B(j);
        qd = qd + beta{j} * dB(j);
        qdd = qdd + beta{j} * ddB(j);
    end
end
