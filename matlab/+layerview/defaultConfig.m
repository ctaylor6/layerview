function config = defaultConfig(projectRoot)
%DEFAULTCONFIG Reproducible settings for the original coin inspection study.
% Coordinate units are millimetres. Raw inputs must already have calibrated
% XY coordinates, or supply the optional image-to-world homography below.
if nargin < 1
    projectRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
end
config.projectRoot = char(projectRoot);
config.inputMode = 'processed';
config.paths.processedScan = fullfile(projectRoot, 'data', 'processed', 'filtered_scan.ply');
config.paths.featureCAD = fullfile(projectRoot, 'data', 'cad', 'A_coin_w6_test.STL');
config.paths.baseCAD = fullfile(projectRoot, 'data', 'cad', 'noA_coin_w6_test.STL');
config.paths.filledCAD = fullfile(projectRoot, 'data', 'cad', 'filled_A_coin_w8_test.STL');
config.paths.calibrationImage = fullfile(projectRoot, 'data', 'calibration', 'checkerboardwarp.png');
config.paths.rawReference = '';
config.paths.rawScan = '';
config.outputName = 'processed_demo';
config.writeOutputs = true;
config.makePlots = true;

config.sampling.seed = 42;
config.sampling.pointsPerSquareMM = 1000;
config.sampling.maxPoints = 200000;
config.sampling.floorTolerance = 0.01;

% CoinProcessing.mlx's repair-scan settings. Earlier baseline experiments
% used zSign = -1; do not assume the same convention for every acquisition.
config.raw.homography = [];
config.raw.zSign = 1;
config.raw.flipY = true;
config.raw.topReferenceCount = 125000;
config.raw.lowerHeightScale = 1;
config.raw.firstNumNeighbors = 50;
config.raw.firstThreshold = 0.04;
config.raw.finalNumNeighbors = 400;
config.raw.finalThreshold = 1.1;

config.registration.metric = 'pointToPlane';
config.registration.maxIterations = 1000;
config.registration.tolerance = [1e-4, 1e-4];
config.registration.initializationMaxPoints = 10000;
config.registration.scoreNeighbors = 10;

% Optional reproduction of ConProcessingReconstruction.m. These are the
% original experiment's slice/threshold values, not general repair settings.
config.reconstruction.enabled = false;
config.reconstruction.sliceZ = [0.15, 0.16];
config.reconstruction.filledCADMinX = 1.2;
config.reconstruction.distanceThreshold = 0.8;
config.reconstruction.extrusionHeight = 1.2 - 0.61;
end
