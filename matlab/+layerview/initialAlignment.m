function [transform, info] = initialAlignment(moving, fixed, options)
%INITIALALIGNMENT PCA initialization with explicit proper-rotation candidates.
% Original scripts compared principal-axis orientations 180 degrees apart.
% Evaluate the PCA sign choices without reflecting the scan or referencing
% an uninitialized variable. The identity orientation is also a candidate.
moving = finitePoints(moving);
fixed = finitePoints(fixed);
movingCenter = mean(moving, 1);
fixedCenter = mean(fixed, 1);
movingBasis = pca(moving);
fixedBasis = pca(fixed);
assert(size(movingBasis, 2) == 3 && size(fixedBasis, 2) == 3, ...
    'layerview:DegenerateCloud', 'PCA requires a three-dimensional coordinate basis.');
validateattributes(options.initializationMaxPoints, {'numeric'}, {'scalar', 'integer', '>=', 6});
validateattributes(options.scoreNeighbors, {'numeric'}, {'scalar', 'integer', 'positive'});
movingScore = boundedPoints(moving, options.initializationMaxPoints);
fixedScore = boundedPoints(fixed, options.initializationMaxPoints);
rotations = eye(3);
for sx = [-1, 1]
    for sy = [-1, 1]
        for sz = [-1, 1]
            rowRotation = movingBasis*diag([sx, sy, sz])*fixedBasis.';
            if det(rowRotation) > 0
                rotations(:, :, end+1) = rowRotation; %#ok<AGROW>
            end
        end
    end
end
scores = zeros(size(rotations, 3), 1);
k = min(options.scoreNeighbors, size(movingScore, 1));
for n = 1:size(rotations, 3)
    rowRotation = rotations(:, :, n);
    translation = fixedCenter-movingCenter*rowRotation;
    candidate = movingScore*rowRotation+translation;
    [~, distances] = knnsearch(candidate, fixedScore, 'K', k);
    scores(n) = mean(distances(:));
end
[~, best] = min(scores);
rowRotation = rotations(:, :, best);
translation = fixedCenter-movingCenter*rowRotation;
% rigidtform3d uses premultiplication; points above use MATLAB row vectors.
transform = rigidtform3d(rowRotation.', translation);
info.candidateScores = scores;
info.selectedCandidate = best;
info.identitySelected = best == 1;
info.rotationDeterminant = det(rowRotation);
info.movingCenter = movingCenter;
info.fixedCenter = fixedCenter;
end

function points = finitePoints(points)
validateattributes(points, {'numeric'}, {'2d', 'ncols', 3, 'finite'});
assert(size(points, 1) >= 6, 'layerview:EmptyCloud', 'At least six finite points are required.');
points = double(points);
assert(rank(points-mean(points, 1)) >= 2, 'layerview:DegenerateCloud', 'Collinear or coincident clouds cannot determine an orientation.');
end

function points = boundedPoints(points, maximum)
if size(points, 1) > maximum
    points = points(round(linspace(1, size(points, 1), maximum)), :);
end
end
