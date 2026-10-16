classdef TestRepairMesh < matlab.unittest.TestCase
    methods (Test)
        function convexPrismIsClosedAndHasExpectedVolume(testCase)
            [mesh, checks] = layerview.extrudeConvexRegion([0, 0; 2, 0; 2, 3; 0, 3; 1, 1], 0.5);
            testCase.verifyTrue(checks.passed);
            testCase.verifySize(mesh.Points, [8, 3]);
            testCase.verifySize(mesh.ConnectivityList, [12, 3]);
            testCase.verifyEqual(checks.signedVolume, 3, 'AbsTol', 1e-12);
            testCase.verifyGreaterThan(checks.minimumTriangleArea, 0);
        end
        function missingFaceIsRejected(testCase)
            mesh = layerview.extrudeConvexRegion([0, 0; 2, 0; 2, 3; 0, 3], 1);
            checks = layerview.validateMesh(mesh.Points, mesh.ConnectivityList(2:end, :));
            testCase.verifyFalse(checks.passed);
        end
        function reversedWindingIsRejected(testCase)
            mesh = layerview.extrudeConvexRegion([0, 0; 2, 0; 2, 3; 0, 3], 1);
            checks = layerview.validateMesh(mesh.Points, mesh.ConnectivityList(:, [1, 3, 2]));
            testCase.verifyFalse(checks.passed);
            testCase.verifyLessThan(checks.signedVolume, 0);
        end
        function degenerateTriangleIsRejected(testCase)
            mesh = layerview.extrudeConvexRegion([0, 0; 2, 0; 2, 3; 0, 3], 1);
            faces = [mesh.ConnectivityList; 1, 1, 2];
            checks = layerview.validateMesh(mesh.Points, faces);
            testCase.verifyFalse(checks.passed);
        end
        function collinearRegionIsRejected(testCase)
            testCase.verifyError(@() layerview.extrudeConvexRegion([0, 0; 1, 0; 2, 0], 1), 'layerview:DegenerateRegion');
        end
    end
end
