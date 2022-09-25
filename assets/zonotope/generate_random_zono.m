%% generate random zonotopes
close all;

NumOfZono = 5;
C = [];
G = {};
% randomize
for i = 1:NumOfZono
    % random centers in 3D space
    C1 = randi([-3, 3]) .* rand(1, 3);
    C = [C; C1];

    % random generators
    G1 = (-0.5 + rand(1, 3));
    G2 = (-0.5 + rand(1, 3));
    G3 = (-0.5 + rand(1, 3));
    G_all = [G1; G2; G3];
    G = [G, {G_all}];
end

% generate zonotope vertices
V = []
for i = 1:NumOfZono
    ver = zono2vertices(C(i, :), cell2mat(G(i)));
    V = [V, {ver'}];
    figure(i); grid on;

    % save convex hull as .STL
    DT = delaunayTriangulation(ver(1, :)', ver(2, :)', ver(3, :)');
    tetramesh(DT, 'FaceAlpha', 0.3);
    [K,v] = convexHull(DT);
    stlwrite(append('meshes/zonomesh_', num2str(i), '.stl'), K, ver')
end

% save vertices matrix
writecell(V, 'vertices.csv')