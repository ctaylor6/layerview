function [mesh, validation] = extrudeConvexRegion(xy, height)
%EXTRUDECONVEXREGION Closed convex-hull extrusion from the original prototype.
% Convexification cannot preserve holes or concave defect boundaries.
validateattributes(xy, {'numeric'}, {'2d', 'ncols', 2, 'finite', 'nonempty'});
validateattributes(height, {'numeric'}, {'scalar', 'positive', 'finite'});
xy = unique(double(xy), 'rows');
assert(size(xy, 1) >= 3 && rank(xy-mean(xy, 1)) == 2, ...
    'layerview:DegenerateRegion', 'At least three noncollinear XY points are required.');
indices = convhull(xy(:, 1), xy(:, 2));
% convhull already repeats the first point; retain one vertex per corner.
boundary = xy(indices(1:end-1), :);
n = size(boundary, 1);
next = [2:n, 1];
twiceArea = sum(boundary(:, 1).*boundary(next, 2)-boundary(next, 1).*boundary(:, 2));
if twiceArea < 0, boundary = flipud(boundary); end
vertices = [boundary, zeros(n, 1); boundary, height*ones(n, 1)];
faces = zeros(4*n-4, 3);
for k = 1:n
    j = mod(k, n)+1;
    faces(2*k-1:2*k, :) = [k, j, n+j; k, n+j, n+k];
end
offset = 2*n;
for k = 2:n-1
    faces(offset+1, :) = [1, k+1, k];
    faces(offset+2, :) = [n+1, n+k, n+k+1];
    offset = offset+2;
end
validation = layerview.validateMesh(vertices, faces);
assert(validation.passed, 'layerview:InvalidRepairMesh', '%s', strjoin(validation.issues, ' '));
mesh = triangulation(faces, vertices);
end
