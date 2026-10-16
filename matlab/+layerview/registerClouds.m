function registration = registerClouds(moving, fixed, options)
%REGISTERCLOUDS PCA-initialized MATLAB ICP; the scan is the moving cloud.
% This integrates pcregistericp; it is not an independent ICP optimizer.
[moving, ~] = removeInvalidPoints(moving);
[fixed, ~] = removeInvalidPoints(fixed);
assert(moving.Count >= 6 && fixed.Count >= 6, 'layerview:EmptyCloud', ...
    'Both registration clouds must contain at least six finite points.');
[initial, info] = layerview.initialAlignment(moving.Location, fixed.Location, options);
initialized = pctransform(moving, initial);
[refinement, aligned, rmse] = pcregistericp(initialized, fixed, ...
    'Metric', options.metric, 'MaxIterations', options.maxIterations, ...
    'Tolerance', options.tolerance);
registration.initialTransform = initial;
registration.refinementTransform = refinement;
registration.transform = layerview.composeTransforms(initial, refinement);
registration.initialization = info;
registration.initializedCloud = initialized;
registration.alignedCloud = aligned;
registration.icpRMSE = rmse;
registration.metric = options.metric;
end
