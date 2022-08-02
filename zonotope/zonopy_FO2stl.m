%% read vertices from generated forward occupancy

for step = [1:30]
    for i = [1:7]
        for j = [1:4]
            filename = append("../zonotope/data/zonopy_sim/zonopy_vertices_step", num2str(step), "_link", num2str(i), "_num", num2str(j), ".csv");
            vertices = readmatrix(filename);
            DT = delaunayTriangulation(vertices);
            % tetramesh(DT, 'FaceAlpha', 0.3);
            [K,v] = convexHull(DT);
            stlwrite(append('meshes/zonopy_mesh', '_step', num2str(step), '_link', num2str(i), '_num', num2str(j), '.stl'), K, vertices);
        end
    end
end