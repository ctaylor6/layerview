function [metrics, residuals] = registrationErrors(scan, cad)
%REGISTRATIONERRORS Euclidean nearest-neighbor residuals in both directions.
% These distances describe geometric mismatch; they are not independently
% calibrated metrology uncertainty or an automatic defect classification.
assert(scan.Count > 0 && cad.Count > 0, 'layerview:EmptyCloud', 'Cannot score an empty point cloud.');
[~, residuals.scanToCAD] = knnsearch(cad.Location, scan.Location, 'K', 1);
[~, residuals.cadToScan] = knnsearch(scan.Location, cad.Location, 'K', 1);
metrics.scanToCAD = summarize(residuals.scanToCAD);
metrics.cadToScan = summarize(residuals.cadToScan);
metrics.symmetricRMSE = sqrt((mean(residuals.scanToCAD.^2)+mean(residuals.cadToScan.^2))/2);
metrics.units = 'mm';
end

function s = summarize(values)
s.count = numel(values);
s.rmse = sqrt(mean(values.^2));
s.mean = mean(values);
s.median = median(values);
s.standardDeviation = std(values);
s.percentile95 = prctile(values, 95);
s.maximum = max(values);
end
