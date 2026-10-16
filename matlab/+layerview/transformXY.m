function points = transformXY(points, homography)
%TRANSFORMXY Apply the image-to-world homography from homogenomatrix.m.
% Height is preserved. The caller owns calibration correspondence ordering.
validateattributes(points, {'numeric'}, {'2d', 'ncols', 3, 'finite'});
points = double(points);
validateattributes(homography, {'numeric'}, {'size', [3, 3], 'finite'});
assert(rank(homography) == 3, 'layerview:SingularHomography', 'Homography must be nonsingular.');
q = double(homography) * [points(:, 1:2).'; ones(1, size(points, 1))];
scale = max(1, max(abs(q), [], 1));
assert(all(abs(q(3, :)) > eps.*scale), ...
    'layerview:HomographyAtInfinity', 'Homography maps at least one point to infinity.');
points(:, 1:2) = (q(1:2, :)./q(3, :)).';
end
