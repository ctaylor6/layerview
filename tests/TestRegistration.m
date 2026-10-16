classdef TestRegistration < matlab.unittest.TestCase
    methods (Test)
        function icpRecoversSyntheticRigidTransform(testCase)
            stream = RandStream('mt19937ar', 'Seed', 8);
            directions = randn(stream, 700, 3);
            fixed = directions./vecnorm(directions, 2, 2).*[4, 2, 1];
            known = rigidtform3d([7, -9, 15], [0.4, -0.3, 0.2]);
            moving = transformPointsInverse(known, fixed);
            config = layerview.defaultConfig();
            config.registration.scoreNeighbors = 1;
            config.registration.maxIterations = 100;
            registration = layerview.registerClouds(pointCloud(moving), pointCloud(fixed), config.registration);
            transformed = transformPointsForward(registration.transform, moving);
            testCase.verifyEqual(transformed, registration.alignedCloud.Location, 'AbsTol', 1e-9);
            testCase.verifyLessThan(sqrt(mean(sum((transformed-fixed).^2, 2))), 1e-6);
        end
        function directionalResidualsExposeMissingCoverage(testCase)
            cad = pointCloud([0, 0, 0; 1, 0, 0; 10, 0, 0]);
            scan = pointCloud([0, 0, 0; 1, 0, 0]);
            [metrics, residuals] = layerview.registrationErrors(scan, cad);
            testCase.verifyEqual(metrics.scanToCAD.rmse, 0);
            testCase.verifyEqual(residuals.cadToScan, [0; 0; 9]);
            testCase.verifyGreaterThan(metrics.cadToScan.rmse, 0);
        end
        function emptyCloudIsRejectedBeforePCA(testCase)
            config = layerview.defaultConfig();
            testCase.verifyError(@() layerview.initialAlignment(zeros(0, 3), rand(10, 3), config.registration), 'layerview:EmptyCloud');
        end
    end
end
