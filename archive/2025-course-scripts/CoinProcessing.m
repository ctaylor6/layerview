%% ME 495 Project 2-24-25 CDT

%% Step 1: Examine geometry of object without feature

clc; clear; close all;
% Randomly Sampeling Surface Points

% Read the STL file
stlData = stlread('noA_coin_w6_test.stl');
vertices = stlData.Points;
faces = stlData.ConnectivityList;

% Number of points per face (increase for higher density)
numSamplesPerFace = 50;

% Initialize array to hold sampled points
sampledPoints = [];

for i = 1:size(faces, 1)
    % Get triangle vertices
    v1 = vertices(faces(i, 1), :);
    v2 = vertices(faces(i, 2), :);
    v3 = vertices(faces(i, 3), :);

    % Compute triangle area using cross product
    triArea = 0.5 * norm(cross(v2 - v1, v3 - v1));

    % Sample points proportional to triangle area
    numSamples = max(1, round(numSamplesPerFace * triArea)); % More for larger triangles

    for j = 1:numSamples
        % Random barycentric coordinates
        a = rand();
        b = rand();
        if (a + b > 1) % Ensure inside the triangle
            a = 1 - a;
            b = 1 - b;
        end
        c = 1 - a - b;

        % Compute random point on the triangle
        p = a * v1 + b * v2 + c * v3;

        if p(1) > 0 % Adjust threshold if needed
            sampledPoints = [sampledPoints; p];
        end

    end
end

sampledPointsmatrix = [sampledPoints(:,1), sampledPoints(:,3), sampledPoints(:,2)];

% Plot the sampled surface points
clc; close all

sampledPointsmatrix = [sampledPoints(:,3), sampledPoints(:,2), sampledPoints(:,1)];

% Point cloud of Part and center
ptCloudUndef = pointCloud(sampledPointsmatrix);
pcshow(ptCloudUndef)
title('Part Surface Point Cloud Data')

tol = 0.01;
zfloor = max(sampledPoints(:,1)) + tol; % Perscribed from STL File


%% Step 2: Subtract the mean of the scan

clc;
%addpath('Baseline Data')
%addpath('Broken Parts')
addpath('Repair Homog')
% Load data
filename = '2_1.xyz';
[x, y, z] = xyzread(filename);
base = [x, y, 1*z];

filename2 = '2_2.xyz';
[x2, y2, z2] = xyzread(filename2);
scan2 = [x2, y2, z2];

scan2(:,3) = scan2(:,3) - mean(base(:,3)); % Remove offset
scan2(:,3) = scan2(:,3).*1; % Flip

% Denoise the subtracted data
ptCloudmfg = pointCloud(scan2);
ptCloudOut = pcdenoise(ptCloudmfg);
pcshow(ptCloudOut)
axis off
axis equal

%% Step 3: Load in STL Point Cloud

% Randomly Sampeling Surface Points
% Read the STL file
stlData = stlread('A_coin_w6_test.stl');
vertices = stlData.Points;
faces = stlData.ConnectivityList;

% Number of points per face (increase for higher density)
numSamplesPerFace = 1000;

% Initialize array to hold sampled points
sampledPoints = [];

for i = 1:size(faces, 1)
    % Get triangle vertices
    v1 = vertices(faces(i, 1), :);
    v2 = vertices(faces(i, 2), :);
    v3 = vertices(faces(i, 3), :);

    % Compute triangle area using cross product
    triArea = 0.5 * norm(cross(v2 - v1, v3 - v1));

    % Sample points proportional to triangle area
    numSamples = max(1, round(numSamplesPerFace * triArea)); % More for larger triangles

    for j = 1:numSamples
        % Random barycentric coordinates
        a = rand();
        b = rand();
        if (a + b > 1) % Ensure inside the triangle
            a = 1 - a;
            b = 1 - b;
        end
        c = 1 - a - b;

        % Compute random point on the triangle
        p = a * v1 + b * v2 + c * v3;

        if p(1) > zfloor % Adjust threshold if needed
            sampledPoints = [sampledPoints; p];
        end

    end
end

sampledPointsmatrix = [sampledPoints(:,1), sampledPoints(:,3), sampledPoints(:,2)];

% Plot the sampled surface points
clc;

sampledPointsmatrix = [sampledPoints(:,3), sampledPoints(:,2), sampledPoints(:,1)];

% Point cloud of Part and center
ptCloudUndef = pointCloud(sampledPointsmatrix);
xyzData2 = ptCloudUndef.Location;  % Extract [X, Y, Z] coordinates
xyzData23 = xyzData2 - mean(xyzData2);
ptCloudSTL = pointCloud(xyzData23);
pcshow(ptCloudSTL)
title('Part Point Cloud')
xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');
ax = gca; ax.FontSize = 20;
axis equal


%% Step 4:  Filter Noise, remove z offset, and outliers

clc;
numNeighbors = 50;  % Number of neighbors to consider
stdThreshold = 0.04; % Standard deviation multiplier

[~, inlierIndices] = pcdenoise(ptCloudOut, 'NumNeighbors', numNeighbors, 'Threshold', stdThreshold);
filteredCloud = select(ptCloudOut, inlierIndices);
filteredCloud = pcdenoise(filteredCloud);

base(:,3) = base(:,3) - mean(base(:,3));

ylowerlimit = median(maxk(base(:,3),125000));

ylowerlimitscale =1

xyzData = filteredCloud.Location;  % Extract [X, Y, Z] coordinates
nozfinalpart = xyzData(xyzData(:,3) > ylowerlimit*ylowerlimitscale & xyzData(:,3)< ylowerlimit*ylowerlimitscale + (max(sampledPoints(:,1))-zfloor), :);
nozfinalpart22 = nozfinalpart - mean(nozfinalpart);
ptCloudOut2w = pointCloud(nozfinalpart22);

ptCloud = ptCloudOut2w;
numNeighbors = 400; % Number of neighbors to consider
threshold = 1.1 % Standard deviation multiplier
[ptCloudFiltered, inlierIndices] = pcdenoise(ptCloud, 'NumNeighbors', numNeighbors, 'Threshold', threshold);

gridSize = 0.1; % Adjust for more/less smoothing
smoothedPC = pcdownsample(ptCloudFiltered, 'gridAverage', gridSize);

pcshow(ptCloudFiltered);
%title('Filtered Point Cloud (Outliers Removed)');
xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');
axis off
ax = gca; ax.FontSize = 30;
ptCloudOut2 = ptCloudFiltered;


%% Step 5:  Roatate STLs to Match

clc;
% Load the fixed and moving point clouds
fixedPC = ptCloudSTL;  % The fixed reference object
movingPC = ptCloudOut2; % The object to be rotated

% Extract XYZ coordinates
fixedXYZ = fixedPC.Location;
movingXYZ1 = movingPC.Location;

movingXYZ = movingXYZ1.*[1, -1, 1];

% Compute PCA for fixed object (Reference)
fixedCoeff = pca(fixedXYZ(:,1:3)); % Ignore Z-axis, use XY only
fixedDir = fixedCoeff(:,1);  % First principal component (dominant axis)
fixedNormal = fixedCoeff(:,3); % Third principal component (normal direction)

% Compute PCA for moving object
movingCoeff = pca(movingXYZ(:,1:3));
movingDir = movingCoeff(:,1); % First principal component
movingNormal = movingCoeff(:,3); % Third principal component (normal direction)

% Compute rotation angle in XY plane
theta = atan2d(movingDir(2), movingDir(1)) - atan2d(fixedDir(2), fixedDir(1));

% Create 2D rotation matrix (counterclockwise rotation)
R1 = [cosd(-theta), -sind(-theta), 0;  % Rotation in XY plane
     sind(-theta), cosd(-theta), 0;
     0, 0, 1];
R2 = [cosd(-theta+180), -sind(-theta+180), 0; sind(-theta+180), cosd(-theta+180), 0; 0, 0, 1]; % 180° correction

% Apply both rotations
alignedXYZ1 = (R1 * movingXYZ')';
alignedXYZ2 = (R2 * movingXYZ')';

% k-nearest neighbor distance sum
k = 10; % Number of neighbors
Mdl1 = KDTreeSearcher(alignedXYZ1);
Mdl2 = KDTreeSearcher(alignedXYZ2);
[~, dist1] = knnsearch(Mdl1, fixedXYZ, 'K', k);
[~, dist2] = knnsearch(Mdl2, fixedXYZ, 'K', k);

% Compute average distances
avgDist1 = mean(dist1(:));
avgDist2 = mean(dist2(:));

% Check if the object is upside down (flipped across Z)
if dot(fixedNormal, movingNormal) < 0
    alignedXYZ(:,3) = -alignedXYZ(:,3); % Flip Z values
end

% Select the better alignment
if avgDist2 < avgDist1
    alignedXYZ = alignedXYZ2;
else
    alignedXYZ = alignedXYZ1;
end

% Create a new point cloud with the best-aligned points
alignedPC = pointCloud(alignedXYZ, 'Color', movingPC.Color);

% Display results
figure;
subplot(2,1,1);
pcshowpair(movingPC, fixedPC);
title('Before PCA Alignment');

subplot(2,1,2);
pcshowpair(alignedPC, fixedPC);
title('After PCA Alignment');
pcwrite(alignedPC, 'filteredPC1.ply');


%% Step 7:  Align and Visualize the Two Point Clouds

% Load in the 2 Point Clouds

ptCloud2 = fixedPC; % Target (fixed) point cloud
ptCloud1 = alignedPC; % Source (moving) point cloud

%% Step 2: Visualize the Original 2 Point Clouds
figure;
subplot(2,2,1);
pcshow(ptCloud1);
title('Target Point Cloud (Original)');
xlabel('X'); ylabel('Y'); zlabel('Z');

subplot(2,2,2);
pcshow(ptCloud2);
title('Source Point Cloud (Original)');
xlabel('X'); ylabel('Y'); zlabel('Z');

%% Step 3: Initial Alignment of the Point Clouds Using PCA
[coeff1,~,~] = pca(ptCloud1.Location);
[coeff2,~,~] = pca(ptCloud2.Location);
initialRotation = coeff1 * coeff2'; % Rough alignment
tformInit = affine3d([initialRotation, [0;0;0]; 0 0 0 1]);
ptCloud2 = pctransform(ptCloud2, tformInit);

%% Step 4: Apply ICP Registration
[tform, ptCloudAligned, rmse] = pcregistericp(ptCloud2, ptCloud1, ...
    'Metric', 'pointToPlane', 'MaxIterations', 1000, 'Tolerance', [0.0001, 0.0001]);

%% Step 5: Compute Registration Errors
if isempty(ptCloud1.Location) || isempty(ptCloudAligned.Location)
    error('One of the point clouds is empty after filtering.');
end

% Find nearest neighbors in ptCloud1 for each point in ptCloudAligned
[indices, distances] = knnsearch(ptCloud1.Location, ptCloudAligned.Location, 'K', 1);

% Compute Euclidean distances (errors)
errors = vecnorm(ptCloudAligned.Location - ptCloud1.Location(indices, :), 2, 2);

%% Step 6: Visualize the Point Clouds Before & After Alignment
subplot(2,2,3);
pcshowpair(ptCloud1, ptCloud2);
title('Before ICP Alignment');
xlabel('X'); ylabel('Y'); zlabel('Z');

subplot(2,2,4);
pcshowpair(ptCloud1, ptCloudAligned);
title('After ICP Alignment');
xlabel('X'); ylabel('Y'); zlabel('Z');

%% Step 7: Registration Error Plot
figure;
pcshow(ptCloudAligned.Location, errors, 'MarkerSize', 50);
colormap jet;
colorbar;
clim([0, max(errors)]); % Set the color scale
%title('Registration Errors (Euclidean Distance)');
xlabel('X'); ylabel('Y'); zlabel('Z');
axis off


%% Step 8: Residual Error Distribution (Histogram)
figure;
histogram(errors, 50); % Plot histogram of the errors
title('Residual Errors Distribution After ICP');
xlabel('Error (Euclidean Distance)');
ylabel('Frequency');
ax = gca; ax.FontSize = 20;
grid on;

std(errors)

%% Step 9: Display Transformation and RMSE
disp('Estimated Transformation Matrix:');
disp(tform.T);
disp(['Final Registration RMSE: ', num2str(rmse)]);
