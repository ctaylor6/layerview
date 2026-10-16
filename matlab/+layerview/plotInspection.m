function plotInspection(result, filename)
%PLOTINSPECTION Compact before/after and geometric-residual visualization.
fig = figure('Visible', 'off', 'Color', 'white', 'Position', [100, 100, 1500, 460]);
cleanup = onCleanup(@() close(fig)); %#ok<NASGU>
tiledlayout(fig, 1, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
nexttile;
pcshowpair(result.registration.initializedCloud, result.referenceCAD, 'MarkerSize', 15);
title('After PCA initialization');
axis equal; xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');
nexttile;
pcshowpair(result.registration.alignedCloud, result.referenceCAD, 'MarkerSize', 15);
title('After ICP registration');
axis equal; xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');
nexttile;
pcshow(result.registration.alignedCloud.Location, result.residuals.scanToCAD, 'MarkerSize', 15);
title('Scan-to-CAD distance (mm)');
axis equal; xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');
colormap(parula); colorbar;
exportgraphics(fig, filename, 'Resolution', 160);
end
