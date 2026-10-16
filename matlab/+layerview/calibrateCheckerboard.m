function [homography, calibration] = calibrateCheckerboard(imageFilename, orderedWorldPoints)
%CALIBRATECHECKERBOARD Estimate XY calibration from explicit paired corners.
% orderedWorldPoints is N-by-2 in mm and MUST follow the order returned by
% detectCheckerboardPoints. Review calibration.imagePoints against the image
% before applying the result; a low residual alone cannot establish orientation.
%
% Original study's 88-corner convention (1.5 mm spacing):
%   [x, y] = meshgrid(0:1.5:15, 10.5:-1.5:0);
%   [H, calibration] = layerview.calibrateCheckerboard(imagePath, [x(:), y(:)]);
%   config.raw.homography = H;
%
% Already calibrated Repair Homog data must not be transformed a second time.
assert(isfile(imageFilename), 'layerview:MissingCalibrationImage', 'Calibration image not found: %s', imageFilename);
image = imread(imageFilename);
if size(image, 3) == 3, image = rgb2gray(image); end
[imagePoints, boardSize] = detectCheckerboardPoints(image);
assert(~isempty(imagePoints), 'layerview:CheckerboardNotFound', 'No checkerboard corners detected in the image.');
assert(size(imagePoints, 1) == size(orderedWorldPoints, 1), 'layerview:CalibrationSizeMismatch', ...
    'Detected %d corners, but received %d world-coordinate pairs.', size(imagePoints, 1), size(orderedWorldPoints, 1));
[homography, calibration] = layerview.estimateHomography(imagePoints, orderedWorldPoints);
calibration.imagePoints = imagePoints;
calibration.worldPoints = orderedWorldPoints;
calibration.boardSize = boardSize;
calibration.imageFilename = char(imageFilename);
calibration.coordinateConvention = 'Image pixel XY -> world XY in mm; height unchanged.';
end
