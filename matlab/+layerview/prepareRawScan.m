function [cloud, info] = prepareRawScan(referenceFile, scanFile, featureHeight, options)
%PREPARERAWSCAN Reproduce CoinProcessing.mlx's repair-scan filtering route.
% No scanner, projector, printer, network, or machine interfaces are invoked.
[reference, info.referenceRead] = layerview.readXYZ(referenceFile);
[scan, info.scanRead] = layerview.readXYZ(scanFile);
if ~isempty(options.homography)
    reference = layerview.transformXY(reference, options.homography);
    scan = layerview.transformXY(scan, options.homography);
end
assert(ismember(options.zSign, [-1, 1]), 'layerview:InvalidZSign', 'zSign must be +1 or -1.');
validateattributes(featureHeight, {'numeric'}, {'scalar', 'positive', 'finite'});
validateattributes(options.topReferenceCount, {'numeric'}, {'scalar', 'integer', 'positive'});
info.referenceMeanZ = mean(reference(:, 3));
scan(:, 3) = (scan(:, 3)-info.referenceMeanZ)*options.zSign;
cloud = pcdenoise(pointCloud(scan));
cloud = pcdenoise(cloud, 'NumNeighbors', neighborCount(options.firstNumNeighbors, cloud), ...
    'Threshold', options.firstThreshold);
cloud = pcdenoise(cloud);
baseZ = (reference(:, 3)-info.referenceMeanZ)*options.zSign;
count = min(options.topReferenceCount, numel(baseZ));
info.lowerHeight = median(maxk(baseZ, count))*options.lowerHeightScale;
info.upperHeight = info.lowerHeight+featureHeight;
points = cloud.Location;
keep = points(:, 3) > info.lowerHeight & points(:, 3) < info.upperHeight;
points = points(keep, :);
assert(size(points, 1) >= 6, 'layerview:EmptyScan', 'Height filtering removed nearly all scan points; check units, zSign and height limits.');
cloud = pointCloud(points);
cloud = pcdenoise(cloud, 'NumNeighbors', neighborCount(options.finalNumNeighbors, cloud), ...
    'Threshold', options.finalThreshold);
assert(cloud.Count >= 6, 'layerview:EmptyScan', 'Too few scan points remain after denoising.');
points = cloud.Location;
if options.flipY
    points(:, 2) = -points(:, 2);
end
cloud = pointCloud(points);
info.appliedHomography = options.homography;
info.zSign = options.zSign;
info.flippedY = options.flipY;
info.outputPoints = cloud.Count;
end

function n = neighborCount(requested, cloud)
assert(cloud.Count >= 6, 'layerview:EmptyScan', 'Too few scan points remain for denoising.');
validateattributes(requested, {'numeric'}, {'scalar', 'integer', 'positive'});
n = min(requested, cloud.Count-1);
end
