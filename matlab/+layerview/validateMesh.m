function validation = validateMesh(vertices, faces)
%VALIDATEMESH Check a triangular closed surface before exporting an STL.
% Checks finiteness, indexing, face area, duplicates, manifold edges,
% consistent winding and positive signed volume. Does not detect all
% possible self-intersections in an arbitrary input mesh.
validation.passed = false;
validation.issues = {};
validation.signedVolume = NaN;
validation.minimumTriangleArea = NaN;
if ~isnumeric(vertices) || size(vertices, 2) ~= 3 || isempty(vertices) || any(~isfinite(vertices(:)))
    validation.issues = {'Vertices must be a nonempty finite N-by-3 numeric array.'}; return;
end
if ~isnumeric(faces) || size(faces, 2) ~= 3 || isempty(faces) || ...
        any(~isfinite(faces(:))) || any(faces(:) ~= fix(faces(:))) || ...
        any(faces(:) < 1) || any(faces(:) > size(vertices, 1))
    validation.issues = {'Faces must contain valid integer vertex indices.'}; return;
end
v1 = vertices(faces(:, 1), :); v2 = vertices(faces(:, 2), :); v3 = vertices(faces(:, 3), :);
areas = 0.5*vecnorm(cross(v2-v1, v3-v1, 2), 2, 2);
scale = max(max(vertices, [], 1)-min(vertices, [], 1));
areaTolerance = max(realmin, eps*scale^2*100);
validation.minimumTriangleArea = min(areas);
if any(areas <= areaTolerance)
    validation.issues{end+1} = 'Mesh contains degenerate or near-zero-area triangles.';
end
if size(unique(sort(faces, 2), 'rows'), 1) ~= size(faces, 1)
    validation.issues{end+1} = 'Mesh contains duplicate triangles.';
end
edges = [faces(:, [1, 2]); faces(:, [2, 3]); faces(:, [3, 1])];
[~, ~, groups] = unique(sort(edges, 2), 'rows');
counts = accumarray(groups, 1);
if any(counts ~= 2)
    validation.issues{end+1} = 'Mesh is not closed and two-manifold: an edge does not have exactly two incident faces.';
end
orientation = sign(edges(:, 2)-edges(:, 1));
if any(accumarray(groups, orientation) ~= 0)
    validation.issues{end+1} = 'Adjacent triangles have inconsistent winding.';
end
% Shift the origin near the geometry to reduce cancellation for large offsets.
origin = mean(vertices, 1);
validation.signedVolume = sum(dot(v1-origin, cross(v2-origin, v3-origin, 2), 2))/6;
if validation.signedVolume <= max(realmin, eps*scale^3*100)
    validation.issues{end+1} = 'Mesh has nonpositive or negligible signed volume.';
end
validation.passed = isempty(validation.issues);
end
