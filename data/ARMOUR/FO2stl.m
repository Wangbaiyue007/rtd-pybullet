%% read vertices from generated forward occupancy and save as stl
clear; clc;
path = '../../assets/zonotope/ARMOUR/3/';
delete(append(path, '*'));
for step = [1:83]
    for link = [1:7]
        vertices_slc = [];
        filename1_slc = append("reachsets_3/step_", num2str(step), "_link_", num2str(link-1), ".txt");
        filename2_slc = append("reachsets_3/step_", num2str(step), "_link_", num2str(link), ".txt");
        vertices_start_slc = readmatrix(filename1_slc);
        vertices_end_slc = readmatrix(filename2_slc);
        vertices_slc = [vertices_start_slc; vertices_end_slc];
        [K_slc,v_slc] = convhull(vertices_slc(:,1),vertices_slc(:,2),vertices_slc(:,3));
        % trisurf(K_slc,vertices_slc(:,1), vertices_slc(:,2), vertices_slc(:,3), 'FaceColor', 'cyan', 'FaceAlpha', 0.5);
        stlwrite(append(path, 'step_', num2str(step), '_link_', num2str(link), '.stl'), K_slc, vertices_slc);
    end
end