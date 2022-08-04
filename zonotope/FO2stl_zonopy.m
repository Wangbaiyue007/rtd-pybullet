%% read vertices from generated forward occupancy
clear; clc;
for step = [1:30]
    for i = [1:7]
        vertices = [];
        for j = [1:100]
            filename = append("../data/ARMTD_zonopy/zonopy_sim/zonopy_vertices_step", num2str(step), "_link", num2str(i), "_num", num2str(j), ".csv");
            ver = readmatrix(filename);
            vertices = [vertices; ver];
        end
        % generate convex hull for each link at each planning step
        [K,v] = convhull(vertices(:,1),vertices(:,2),vertices(:,3));
        % to see what the convex hull looks like
        % figure(i);
        % trisurf(K,vertices(:,1), vertices(:,2), vertices(:,3),'FaceColor','cyan');
        stlwrite(append('meshes/zonopy_mesh', '_step', num2str(step), '_link', num2str(i), '.stl'), K, vertices);
    end
end