function writeOutputs(result)
%WRITEOUTPUTS Save inspection artifacts only under the configured outputs/.
out = result.outputDirectory;
expected = fullfile(result.config.projectRoot, 'outputs', char(result.config.outputName));
assert(strcmp(out, expected), 'layerview:InvalidOutputPath', 'Output directory does not match the configured outputs path.');
if ~isfolder(out), mkdir(out); end
pcwrite(result.registration.alignedCloud, fullfile(out, 'aligned_scan.ply'), 'Encoding', 'binary');
pcwrite(result.referenceCAD, fullfile(out, 'reference_cad.ply'), 'Encoding', 'binary');
writetable(table(result.residuals.scanToCAD, 'VariableNames', {'distance_mm'}), ...
    fullfile(out, 'scan_to_cad_residuals.csv'));
writetable(table(result.residuals.cadToScan, 'VariableNames', {'distance_mm'}), ...
    fullfile(out, 'cad_to_scan_residuals.csv'));

report.config = result.config;
report.preprocessing = result.preprocessing;
report.referenceSampling = result.referenceSampling;
report.metrics = result.metrics;
report.initialization = result.registration.initialization;
report.initialTransform = result.registration.initialTransform.A;
report.refinementTransform = result.registration.refinementTransform.A;
report.scanToCADTransform = result.registration.transform.A;
report.transformConvention = ['Premultiply homogeneous column coordinates: CAD = scanToCADTransform * [scan; 1]. ' ...
    'For raw input this starts after calibration, baseline subtraction and optional Y reflection. CAD is centered by referenceSampling.centerOffset.'];
report.interpretation = 'Geometric registration residuals in mm; not validated dimensional uncertainty or an automatic defect classification.';
if isfield(result, 'reconstruction')
    assert(result.reconstruction.validation.passed, 'layerview:InvalidRepairMesh', 'Refusing to export an invalid repair mesh.');
    stlwrite(result.reconstruction.mesh, fullfile(out, 'experimental_repair.stl'));
    report.reconstruction.validation = result.reconstruction.validation;
    report.reconstruction.note = result.reconstruction.note;
end
fid = fopen(fullfile(out, 'report.json'), 'w');
assert(fid ~= -1, 'layerview:OutputWriteFailed', 'Could not create report.json.');
cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid, '%s\n', jsonencode(report, 'PrettyPrint', true));
if result.config.makePlots
    layerview.plotInspection(result, fullfile(out, 'inspection.png'));
end
end
