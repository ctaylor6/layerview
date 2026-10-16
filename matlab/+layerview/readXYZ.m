function [points, info] = readXYZ(filename)
%READXYZ Read plain numeric XYZ columns without third-party helper code.
assert(isfile(filename), 'layerview:MissingScan', 'XYZ input not found: %s', filename);
points = readmatrix(filename, 'FileType', 'text');
assert(isnumeric(points) && size(points, 2) == 3, ...
    'layerview:InvalidXYZ', 'Expected exactly three numeric columns in %s.', filename);
valid = all(isfinite(points), 2);
info.inputRows = size(points, 1);
info.removedNonfiniteRows = sum(~valid);
points = double(points(valid, :));
assert(size(points, 1) >= 6, 'layerview:EmptyScan', 'Too few finite XYZ points in %s.', filename);
end
