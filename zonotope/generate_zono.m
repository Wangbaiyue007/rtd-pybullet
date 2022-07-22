%% generate random zonotopes

NumOfZono = 5;
C = [];
G = {};
% randomize
for i = 1:NumOfZono
    % random centers in 3D space
    C1 = randi([-5, 5]) .* rand(1, 3);
    C = [C; C1];

    % random generators
    G1 = randi([-1, 1]) .* rand(1, 3);
    G2 = randi([-1, 1]) .* rand(1, 3);
    G3 = randi([-1, 1]) .* rand(1, 3);
    G_all = [G1; G2; G3];
    G = [G, {G_all}]
end

% generate zonotope vertices
V = []
for i = 1:NumOfZono
    ver = zono2vertices(C(i, :), cell2mat(G(i)));
    V = [V, {ver'}];
end

% save vertices matrix
writecell(V, 'data/vertices.csv')