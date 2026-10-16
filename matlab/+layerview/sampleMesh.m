function [points, info] = sampleMesh(vertices, faces, options)
%SAMPLEMESH Area-weighted barycentric samples with a local random stream.
% Based on MEProjectScript1.m / CoinProcessing.mlx. A bounded total budget
% replaces unbounded array growth; no global RNG state is changed.
validateattributes(vertices, {'numeric'}, {'2d', 'ncols', 3, 'finite', 'nonempty'});
validateattributes(faces, {'numeric'}, {'2d', 'ncols', 3, 'integer', 'positive', 'nonempty'});
assert(all(faces(:) <= size(vertices, 1)), 'layerview:InvalidFaces', 'Face index exceeds the vertex count.');
validateattributes(options.pointsPerSquareMM, {'numeric'}, {'scalar', 'positive', 'finite'});
validateattributes(options.maxPoints, {'numeric'}, {'scalar', 'integer', '>=', 3, 'finite'});
validateattributes(options.seed, {'numeric'}, {'scalar', 'integer', 'nonnegative', '<=', 2^32-1});
v1 = double(vertices(faces(:, 1), :));
v2 = double(vertices(faces(:, 2), :));
v3 = double(vertices(faces(:, 3), :));
areas = 0.5 * vecnorm(cross(v2-v1, v3-v1, 2), 2, 2);
valid = isfinite(areas) & areas > 0;
assert(any(valid), 'layerview:EmptyMesh', 'Mesh has no nondegenerate triangles.');
v1 = v1(valid, :); v2 = v2(valid, :); v3 = v3(valid, :);
areas = areas(valid);
requested = max(1, round(options.pointsPerSquareMM * areas));
assert(all(isfinite(requested)), 'layerview:SamplingOverflow', 'Sampling density or mesh area is too large.');
requestedTotal = sum(requested);
if requestedTotal <= options.maxPoints
    counts = requested;
else
    ideal = options.maxPoints * (requested / requestedTotal);
    counts = floor(ideal);
    [~, order] = sort(ideal-counts, 'descend');
    remaining = options.maxPoints - sum(counts);
    counts(order(1:remaining)) = counts(order(1:remaining)) + 1;
end
points = zeros(sum(counts), 3);
stream = RandStream('mt19937ar', 'Seed', options.seed);
offset = 0;
for k = 1:numel(counts)
    n = counts(k);
    if n == 0, continue; end
    uv = rand(stream, n, 2);
    outside = sum(uv, 2) > 1;
    uv(outside, :) = 1-uv(outside, :);
    points(offset+(1:n), :) = uv(:, 1).*v1(k, :) + ...
        uv(:, 2).*v2(k, :) + (1-sum(uv, 2)).*v3(k, :);
    offset = offset+n;
end
info.requestedPoints = requestedTotal;
info.sampledPoints = size(points, 1);
info.skippedDegenerateFaces = sum(~valid);
info.capped = requestedTotal > options.maxPoints;
info.seed = options.seed;
end
