%% read vertices from generated forward occupancy
clear; clc;
for step = [1:8]
    for i = [1:7]
        vertices = [];
        vertices_slc = [];
        % for j = [1:100]
        filename = append("../../data/ARMTD_zonopy/zonopy_sim/zonopy_vertices_step", num2str(step), "_link", num2str(i), ".csv");
        filename_slc = append("../../data/ARMTD_zonopy/zonopy_sim/zonopy_vertices_slc_step", num2str(step), "_link", num2str(i), ".csv");
        % filename = append("../../data/ARMTD_matlab/matlab_sim/matlab_vertices_step", num2str(step), "_link", num2str(i), ".csv");
        % filename_slc = append("../../data/ARMTD_matlab/matlab_sim/matlab_vertices_slc_step", num2str(step), "_link", num2str(i), ".csv");
        ver = readmatrix(filename);
        vertices = [vertices; ver];
        ver_slc = readmatrix(filename_slc);
        vertices_slc = [vertices_slc; ver_slc];
        % end
        % generate convex hull for each link at each planning step
        [K,v] = convhull(vertices(:,1),vertices(:,2),vertices(:,3));
        [K_slc,v_slc] = convhull(vertices_slc(:,1),vertices_slc(:,2),vertices_slc(:,3));
        % to see what the convex hull looks like
        % figure(i);
        % trisurf(K,vertices(:,1), vertices(:,2), vertices(:,3),'FaceColor','cyan');
%         stlwrite(append('meshes_matlab/matlab_mesh_step', num2str(step), '_link', num2str(i), '.stl'), K, vertices);
%         stlwrite(append('meshes_matlab/matlab_mesh_slc_step', num2str(step), '_link', num2str(i), '.stl'), K_slc, vertices_slc);
        stlwrite(append('meshes_zonopy/zonopy_mesh_step', num2str(step), '_link', num2str(i), '.stl'), K, vertices);
        stlwrite(append('meshes_zonopy/zonopy_mesh_slc_step', num2str(step), '_link', num2str(i), '.stl'), K_slc, vertices_slc);
    end
end