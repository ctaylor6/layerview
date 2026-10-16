function result = run_layerview(config)
%RUN_LAYERVIEW Inspect a coin scan against CAD with PCA-initialized ICP.
%   addpath('matlab'); result = run_layerview();
%
%   config = layerview.defaultConfig();
%   config.inputMode = 'raw';
%   config.paths.rawReference = '/path/to/calibrated/reference.xyz';
%   config.paths.rawScan = '/path/to/calibrated/object.xyz';
%   config.outputName = 'raw_scan';
%   result = run_layerview(config);
%
% All writes go to <projectRoot>/outputs/<outputName>. The default PLY is a
% historical processed scan, so this demonstration does not rerun acquisition
% or raw-scan filtering. Reconstruction is disabled unless explicitly enabled.
if nargin < 1, config = layerview.defaultConfig(); end
assert(ischar(config.outputName) || isstring(config.outputName), ...
    'layerview:InvalidOutputName', 'outputName must be text.');
assert(~isempty(regexp(char(config.outputName), '^[A-Za-z0-9_-]+$', 'once')), ...
    'layerview:InvalidOutputName', 'outputName must contain only letters, numbers, underscores or hyphens.');
assert(isfile(config.paths.baseCAD), 'layerview:MissingCAD', 'Base CAD file not found: %s', config.paths.baseCAD);
baseMesh = stlread(config.paths.baseCAD);
featureFloor = max(baseMesh.Points(:, 1))+config.sampling.floorTolerance;
[referenceCAD, referenceInfo] = layerview.sampleCAD(config.paths.featureCAD, config.sampling, featureFloor);

switch lower(char(config.inputMode))
    case 'processed'
        assert(isfile(config.paths.processedScan), 'layerview:MissingScan', ...
            'Processed scan not found: %s', config.paths.processedScan);
        scan = pcread(config.paths.processedScan);
        originalCount = scan.Count;
        [scan, ~] = removeInvalidPoints(scan);
        preprocessing.mode = 'historical processed PLY; raw filtering and calibration were not rerun';
        preprocessing.inputPoints = originalCount;
        preprocessing.outputPoints = scan.Count;
    case 'raw'
        assert(~isempty(config.paths.rawReference) && ~isempty(config.paths.rawScan), ...
            'layerview:MissingRawPaths', 'Raw mode requires explicit rawReference and rawScan paths.');
        [scan, preprocessing] = layerview.prepareRawScan(config.paths.rawReference, ...
            config.paths.rawScan, referenceInfo.featureHeight, config.raw);
        preprocessing.mode = 'raw XYZ filtering with configured coordinate calibration';
    otherwise
        error('layerview:InvalidInputMode', 'inputMode must be processed or raw.');
end

registration = layerview.registerClouds(scan, referenceCAD, config.registration);
[metrics, residuals] = layerview.registrationErrors(registration.alignedCloud, referenceCAD);
metrics.icpRMSE = registration.icpRMSE;
metrics.icpMetric = registration.metric;
result.config = config;
result.referenceCAD = referenceCAD;
result.referenceSampling = referenceInfo;
result.preprocessedScan = scan;
result.preprocessing = preprocessing;
result.registration = registration;
result.metrics = metrics;
result.residuals = residuals;
result.outputDirectory = fullfile(config.projectRoot, 'outputs', char(config.outputName));
if config.reconstruction.enabled
    % Preserve the slice coordinates used by the original reconstruction
    % experiment: centered preprocessed scan before the new ICP refinement.
    centered = pointCloud(scan.Location-mean(scan.Location, 1));
    result.reconstruction = layerview.reconstructRepair(centered, config);
end
if config.writeOutputs
    layerview.writeOutputs(result);
end
fprintf('Scan -> CAD: %d scan points, %d CAD points\n', scan.Count, referenceCAD.Count);
fprintf('ICP RMSE: %.6g mm; nearest-neighbor scan -> CAD RMSE: %.6g mm\n', ...
    metrics.icpRMSE, metrics.scanToCAD.rmse);
fprintf('Nearest-neighbor CAD -> scan RMSE: %.6g mm\n', metrics.cadToScan.rmse);
if config.writeOutputs, fprintf('Outputs: %s\n', result.outputDirectory); end
end
