%% read vertices from generated forward occupancy and save as stl
clear; clc;
path = '../../assets/zonotope/ARMOUR/1/';
delete(append(path, '*'));
tid = 128;
for step = [1:76]
    for link = [1:8]
        vertices_slc = [];
        c_mat = readmatrix(append("reachsets/step_", num2str(step), "_center.txt"));
        g_mat = readmatrix(append("reachsets/step_", num2str(step), "_radius.txt"));
        c = c_mat((tid-1)*8+link, :)';
        g = g_mat(((tid-1)*8+link-1)*3+1 : ((tid-1)*8+link)*3, :);
        Z = zonotope(c, g);
        Z_v = vertices(Z)';
        [K_slc,v_slc] = convhull(Z_v(:,1), Z_v(:,2), Z_v(:,3));
        % trisurf(K_slc,vertices_slc(:,1), vertices_slc(:,2), vertices_slc(:,3), 'FaceColor', 'cyan', 'FaceAlpha', 0.5);
        stlwrite(append(path, 'step_', num2str(step), '_link_', num2str(link), '.stl'), K_slc, Z_v);
    end
end