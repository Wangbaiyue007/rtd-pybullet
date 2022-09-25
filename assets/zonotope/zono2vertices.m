function [ver] = zono2vertices(center, varargin)
%ZONO2VERTICES Generate vertices from input zonotope
%   The varargin represents generators
for i = 1:length(varargin)
    zmat = [center; varargin{i}];
end

Z = zonotope(zmat');
ver = vertices(Z);

end