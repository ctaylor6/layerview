function [homography, info] = estimateHomography(imagePoints, worldPoints)
%ESTIMATEHOMOGRAPHY SVD solution of paired planar image/world coordinates.
% The DLT formulation follows homography_solve in homogenomatrix.m, with
% coordinate normalization for conditioning. Inputs are N-by-2 arrays in
% matching order; pixel image coordinates map to world coordinates in mm.
validateattributes(imagePoints, {'numeric'}, {'2d', 'ncols', 2, 'finite'});
validateattributes(worldPoints, {'numeric'}, {'size', size(imagePoints), 'finite'});
assert(size(imagePoints, 1) >= 4, 'layerview:InsufficientCalibration', 'At least four paired calibration points are required.');
assert(rank(imagePoints-mean(imagePoints, 1)) == 2 && ...
    rank(worldPoints-mean(worldPoints, 1)) == 2, ...
    'layerview:DegenerateCalibration', 'Calibration points must not be collinear.');
[imageNormalized, imageTransform] = normalizePoints(double(imagePoints));
[worldNormalized, worldTransform] = normalizePoints(double(worldPoints));
n = size(imagePoints, 1);
x = imageNormalized(:, 1); y = imageNormalized(:, 2);
u = worldNormalized(:, 1); v = worldNormalized(:, 2);
z = zeros(n, 1); o = ones(n, 1);
matrix = [-x, -y, -o, z, z, z, u.*x, u.*y, u; ...
    z, z, z, -x, -y, -o, v.*x, v.*y, v];
assert(rank(matrix) >= 8, 'layerview:DegenerateCalibration', 'Calibration correspondences do not determine a homography.');
% A full SVD is needed for four pairs (the 8-by-9 matrix has a null vector
% that an economy SVD would omit). For larger sets economy avoids O(N^2).
if size(matrix, 1) < size(matrix, 2)
    [~, singularValues, vectors] = svd(matrix);
else
    [~, singularValues, vectors] = svd(matrix, 0);
end
normalizedHomography = reshape(vectors(:, end), 3, 3).';
homography = worldTransform\normalizedHomography*imageTransform;
homography = homography/norm(homography, 'fro');
if abs(homography(3, 3)) > 1e-12
    homography = homography/homography(3, 3);
end
projected = layerview.transformXY([double(imagePoints), zeros(n, 1)], homography);
distances = vecnorm(projected(:, 1:2)-worldPoints, 2, 2);
info.pointCount = n;
info.reprojectionRMSE = sqrt(mean(distances.^2));
info.maximumReprojectionError = max(distances);
info.units = 'mm';
info.singularValues = diag(singularValues);
end

function [normalized, transform] = normalizePoints(points)
center = mean(points, 1);
distance = mean(vecnorm(points-center, 2, 2));
assert(distance > 0, 'layerview:DegenerateCalibration', 'Calibration points must span a plane.');
scale = sqrt(2)/distance;
transform = [scale, 0, -scale*center(1); 0, scale, -scale*center(2); 0, 0, 1];
normalized = (points-center)*scale;
end
