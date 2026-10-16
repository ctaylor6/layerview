% Carter Taylor 2-24-2025
clc; clear; close all

C = imread("checkerboardwarp.png");

if size(C,3) == 3
    C = rgb2gray(C);  % Convert to grayscale
end

[imagePoints,boardSize] = detectCheckerboardPoints(C);

%%IDEAL checkerboards

idealcheckerboard_points = [
    0.0, 10.5; 0.0, 9.0; 0.0, 7.5; 0.0, 6.0; 0.0, 4.5; 0.0, 3.0; 0.0, 1.5; 0.0, 0.0;
    1.5, 10.5; 1.5, 9.0; 1.5, 7.5; 1.5, 6.0; 1.5, 4.5; 1.5, 3.0; 1.5, 1.5; 1.5, 0.0;
    3.0, 10.5; 3.0, 9.0; 3.0, 7.5; 3.0, 6.0; 3.0, 4.5; 3.0, 3.0; 3.0, 1.5; 3.0, 0.0;
    4.5, 10.5; 4.5, 9.0; 4.5, 7.5; 4.5, 6.0; 4.5, 4.5; 4.5, 3.0; 4.5, 1.5; 4.5, 0.0;
    6.0, 10.5; 6.0, 9.0; 6.0, 7.5; 6.0, 6.0; 6.0, 4.5; 6.0, 3.0; 6.0, 1.5; 6.0, 0.0;
    7.5, 10.5; 7.5, 9.0; 7.5, 7.5; 7.5, 6.0; 7.5, 4.5; 7.5, 3.0; 7.5, 1.5; 7.5, 0.0;
    9.0, 10.5; 9.0, 9.0; 9.0, 7.5; 9.0, 6.0; 9.0, 4.5; 9.0, 3.0; 9.0, 1.5; 9.0, 0.0;
    10.5, 10.5; 10.5, 9.0; 10.5, 7.5; 10.5, 6.0; 10.5, 4.5; 10.5, 3.0; 10.5, 1.5; 10.5, 0.0;
    12.0, 10.5; 12.0, 9.0; 12.0, 7.5; 12.0, 6.0; 12.0, 4.5; 12.0, 3.0; 12.0, 1.5; 12.0, 0.0;
    13.5, 10.5; 13.5, 9.0; 13.5, 7.5; 13.5, 6.0; 13.5, 4.5; 13.5, 3.0; 13.5, 1.5; 13.5, 0.0;
    15.0, 10.5; 15.0, 9.0; 15.0, 7.5; 15.0, 6.0; 15.0, 4.5; 15.0, 3.0; 15.0, 1.5; 15.0, 0.0];

%getting FPP matrix
addpath('Repair Parts')
filename = '2_2.xyz'; % Adjust to the actual file name

% Open the file
fid = fopen(filename, 'r');

% Read the data
data = textscan(fid, '%f %f %f');
fclose(fid);

% Extract x and y coordinates into a 2xN matrix
FPP = [data{1}'; data{2}'];

%%%HOMOGRAPHY
pin = imagePoints';  % Transpose to 2xN
pout = idealcheckerboard_points'; % Transpose to 2xN

H=homography_solve(pin,pout);
disp('Computed Homography Matrix:');
disp(H);

TF=homography_transform(FPP,H);

%saving new .xyz
data = load(filename);
new_data = TF;
modified_data = [new_data(1,:)' new_data(2,:)' data(:, 3)];
output_filename = '2_2.xyz';
save(output_filename, 'modified_data', '-ascii');

function v = homography_solve(pin, pout)

if ~isequal(size(pin), size(pout))
    error('Points matrices different sizes');
end
if size(pin, 1) ~= 2
    error('Points matrices must have two rows');
end
n = size(pin, 2);
if n < 4
    error('Need at least 4 matching points');
end

% Solve equations using SVD
x = pout(1, :); y = pout(2,:); X = pin(1,:); Y = pin(2,:);
rows0 = zeros(3, n);
rowsXY = -[X; Y; ones(1,n)];
hx = [rowsXY; rows0; x.*X; x.*Y; x];
hy = [rows0; rowsXY; y.*X; y.*Y; y];
h = [hx hy];
if n == 4
    [U, ~, ~] = svd(h);
else
    [U, ~, ~] = svd(h, 'econ');
end
v = (reshape(U(:,9), 3, 3)).';
end


function y = homography_transform(x, v)

q = v * [x; ones(1, size(x,2))];
p = q(3,:);
y = [q(1,:)./p; q(2,:)./p];
end
