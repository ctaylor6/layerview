function result = reconstructRepair(scan, config)
%RECONSTRUCTREPAIR Optional experiment-specific convex repair geometry.
% Follows ConProcessingReconstruction.m: slice scan; align filled CAD;
% retain CAD samples far from scan; convex hull; constant-height extrusion.
options = config.reconstruction;
validateattributes(options.sliceZ, {'numeric'}, {'size', [1, 2], 'finite'});
assert(options.sliceZ(2) > options.sliceZ(1), 'layerview:InvalidRepairSlice', 'sliceZ must contain increasing lower and upper bounds.');
validateattributes(options.distanceThreshold, {'numeric'}, {'scalar', 'positive', 'finite'});
points = scan.Location;
keep = points(:, 3) > options.sliceZ(1) & points(:, 3) < options.sliceZ(2);
assert(sum(keep) >= 6, 'layerview:EmptyRepairSlice', ...
    'Too few points in the configured repair slice; inspect the scan coordinate frame.');
slice = pointCloud(points(keep, :));
[filled, sampling] = layerview.sampleCAD(config.paths.filledCAD, config.sampling, options.filledCADMinX);
registration = layerview.registerClouds(filled, slice, config.registration);
[~, distances] = knnsearch(slice.Location, registration.alignedCloud.Location, 'K', 1);
missing = registration.alignedCloud.Location(distances > options.distanceThreshold, :);
assert(size(missing, 1) >= 3, 'layerview:NoRepairRegion', 'No usable missing region was identified at the configured threshold.');
[mesh, validation] = layerview.extrudeConvexRegion(missing(:, 1:2), options.extrusionHeight);
result.mesh = mesh;
result.validation = validation;
result.missingPoints = missing;
result.filledCADSampling = sampling;
result.registration = registration;
result.note = 'Experimental convex extrusion in the inspection frame; not a validated repair, machine-ready toolpath, or general Boolean subtraction.';
end
