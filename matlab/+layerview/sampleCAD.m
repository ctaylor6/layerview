function [cloud, info] = sampleCAD(filename, options, minimumX)
%SAMPLECAD Sample the STL feature and map original STL X into scan height Z.
assert(isfile(filename), 'layerview:MissingCAD', 'CAD file not found: %s', filename);
mesh = stlread(filename);
[points, info] = layerview.sampleMesh(mesh.Points, mesh.ConnectivityList, options);
points = points(points(:, 1) > minimumX, [3, 2, 1]);
assert(size(points, 1) >= 6, 'layerview:EmptyFeature', 'Too few CAD samples remain above the feature floor.');
info.retainedPoints = size(points, 1);
info.coordinateOrder = [3, 2, 1];
info.centerOffset = mean(points, 1);
info.minimumSTLX = minimumX;
info.featureHeight = max(mesh.Points(:, 1))-minimumX;
cloud = pointCloud(points-info.centerOffset);
end
