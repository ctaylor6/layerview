%% Randomly Sampeling Surface Points
% Read the STL file
clc; clear; close all

stlData = stlread('filled_A_coin_w8_test.stl');
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

        if p(1) > 1.2 % Adjust threshold if needed
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
A_filled_stl = pointCloud(xyzData23);
pcshow(A_filled_stl)
%title('Part Point Cloud')
axis equal
axis off

%% Step 1: Load STL and Convert to Point Cloud
clc
pcd_hollow = pcread('filteredPC1.ply');   % The scanned "A" (point cloud)

ylowerlimit = 0.15;

xyzData = pcd_hollow.Location;  % Extract [X, Y, Z] coordinates
nozfinalpart = xyzData(xyzData(:,3) > ylowerlimit & xyzData(:,3)< ylowerlimit + 0.01,:);
A_filled_pc = pointCloud(nozfinalpart);

% Visualize STL as Point Cloud
figure;
pcshow(A_filled_pc); hold on;
pcshow(A_filled_stl);
title('STL (Blue) and Scanned (Red) as Point Clouds'); axis equal; grid on;

%% ICP
clc; close all

% Load in the 2 Point Clouds
ptCloud2 = A_filled_stl; % Target (fixed) point cloud
ptCloud1 = A_filled_pc; % Source (moving) point cloud

figure;
subplot(2,2,1);
pcshow(ptCloud1);
title('Point Cloud from Fringe Projection');
xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');
ax = gca; ax.FontSize = 20;

subplot(2,2,2);
pcshow(ptCloud2);
title('STL Point Cloud');
xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');
ax = gca; ax.FontSize = 20;

[coeff1,~,~] = pca(ptCloud1.Location);
[coeff2,~,~] = pca(ptCloud2.Location);
initialRotation = coeff1 * coeff2'; % Rough alignment
tformInit = affine3d([initialRotation, [0;0;0]; 0 0 0 1]);
ptCloud2 = pctransform(ptCloud2, tformInit);

% Step 9: Apply ICP Registration
[tform, ptCloudAligned, rmse] = pcregistericp(ptCloud2, ptCloud1, ...
    'Metric', 'pointToPlane', 'MaxIterations', 1000, 'Tolerance', [0.0001, 0.0001]);

% Step 11: Compute Registration Errors
if isempty(ptCloud1.Location) || isempty(ptCloudAligned.Location)
    error('One of the point clouds is empty after filtering.');
end

% Find nearest neighbors in ptCloud1 for each point in ptCloudAligned
[indices, distances] = knnsearch(ptCloud1.Location, ptCloudAligned.Location, 'K', 1);

% Compute Euclidean distances (errors)
errors = vecnorm(ptCloudAligned.Location - ptCloud1.Location(indices, :), 2, 2);

% Step 10: Visualize Point Clouds Before & After Alignment
subplot(2,2,3);
pcshowpair(ptCloud1, ptCloud2);
title('Before ICP Alignment');
axis equal
xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');
ax = gca; ax.FontSize = 20;

subplot(2,2,4);
pcshowpair(ptCloud1, ptCloudAligned);
title('After ICP Alignment');
axis equal
xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');
ax = gca; ax.FontSize = 20;

%% Step 2: Remove Overlapping Points (Boolean Subtraction)
% Define threshold for "closeness" between points
A_filled_pc = ptCloud1;
pcd_hollow = ptCloudAligned;
clc

dist_thresh = 0.8; % Adjust based on scan noise

% Find distances from scanned A to filled A
[idx, dist] = knnsearch(A_filled_pc.Location, pcd_hollow.Location);

% Keep only points in the scan that are NOT close to the filled A
center_points = pcd_hollow.Location(dist > dist_thresh, :);
center_pcd = pointCloud(center_points);

% Visualize Extracted Center Region
figure;
pcshow(center_pcd);
title('Extracted Center Region (Point Cloud)'); axis equal; grid on;
ax = gca; ax.FontSize = 30;

%% Step 1: Define the 2D Shape (Your center extracted points)
% Assume you have the 2D points (e.g., from the boundary of your center region).
% For demonstration purposes, I'll use a simple square (replace with your 2D points).
clc; close all

% Example 2D points for the shape (replace with your actual points)
xyz = center_pcd.Location;
x = xyz(:,1);
y = xyz(:,2);
z = zeros(length(xyz(:,3)),1);

% Get the convex hull (this gives the boundary points)
k = convhull(x, y);

% Extract the boundary points from the convex hull
boundary_x = x(k);
boundary_y = y(k);

% Create a closed loop for the boundary points (close the loop by adding the first point again)
boundary_x = [boundary_x; boundary_x(1)];
boundary_y = [boundary_y; boundary_y(1)];

% Plot the convex hull boundary
figure;
plot(boundary_x, boundary_y, '-r', 'LineWidth', 2);
title('Convex Hull Boundary Around Points');
axis equal;
ax = gca; ax.FontSize = 20;
xlabel('X')
ylabel('Y')
grid on;

%% Make 3D surface and Write to stl
clc; close all

% Extrusion distance (along the z-axis)
extrusion = 1.2-0.61;

% Number of boundary points
n = length(boundary_x);

% Create vertices: bottom layer at z=0 and top layer at z=extrusion
vertices = [ boundary_x(:), boundary_y(:), zeros(n,1);    % bottom vertices
             boundary_x(:), boundary_y(:), extrusion*ones(n,1)]; % top vertices

% Create side faces (quadrilaterals) by connecting corresponding vertices
% Each row here is a quad defined by 4 vertex indices.
sideFaces = [];
for i = 1:n-1
    % bottom_i, bottom_next, top_next, top_i form one quad face
    bottom_i = i;
    bottom_next = i+1;
    top_i = i + n;
    top_next = i+1 + n;

    sideFaces = [sideFaces; bottom_i, bottom_next, top_next, top_i];
end

% Create top and bottom faces as polygons
bottomFace = 1:n;
topFace = n+1:2*n;

% Triangulate faces (STL requires triangles)

% Triangulate side faces (each quad becomes two triangles)
triSideFaces = [];
for i = 1:size(sideFaces,1)
    quad = sideFaces(i,:);
    % Two triangles: [v1,v2,v3] and [v1,v3,v4]
    triSideFaces = [triSideFaces;
                    quad(1) quad(2) quad(3);
                    quad(1) quad(3) quad(4)];
end

% Triangulate the bottom face using a fan method (assumes convex polygon)
bottomTriangles = [];
for i = 2:(length(bottomFace)-1)
    bottomTriangles = [bottomTriangles; bottomFace(1), bottomFace(i), bottomFace(i+1)];
end

% Triangulate the top face (fan method)
topTriangles = [];
for i = 2:(length(topFace)-1)
    topTriangles = [topTriangles; topFace(1), topFace(i), topFace(i+1)];
end

% Combine all triangles
facesTri = [triSideFaces; bottomTriangles; topTriangles];

TR = triangulation(facesTri,vertices);
triplot(TR)

%% Write to STL file using stlwrite
clc
filename = 'extruded_shape.stl';

stlwrite(TR, filename);
disp(['STL file "' filename '" has been created.']);

%% Visualize the triangulated mesh using patch
figure; close all
patch('Vertices', vertices, 'Faces', facesTri, ...
      'FaceColor', 'cyan', 'FaceAlpha', 0.8, 'EdgeColor', 'k');
xlabel('X'); ylabel('Y'); zlabel('Z');
axis equal; grid on;
view(3);
%title('Extruded 3D Shape Mesh');
axis off
