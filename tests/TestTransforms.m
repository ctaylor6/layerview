classdef TestTransforms < matlab.unittest.TestCase
    methods (Test)
        function compositionUsesPhysicalApplicationOrder(testCase)
            first = rigidtform3d([0, 0, 30], [1, 2, 3]);
            second = rigidtform3d([12, -8, 5], [-2, 4, 0.2]);
            points = [0, 0, 0; 1, 2, 3; -2, 1, 4];
            composed = layerview.composeTransforms(first, second);
            expected = transformPointsForward(second, transformPointsForward(first, points));
            testCase.verifyEqual(transformPointsForward(composed, points), expected, 'AbsTol', 1e-12);
        end
        function pcaRecoversKnownRigidPose(testCase)
            stream = RandStream('mt19937ar', 'Seed', 27);
            fixed = rand(stream, 600, 3).*[7, 3, 1];
            known = rigidtform3d([19, -13, 47], [4, -8, 2]);
            moving = transformPointsInverse(known, fixed);
            config = layerview.defaultConfig();
            config.registration.scoreNeighbors = 1;
            [estimated, info] = layerview.initialAlignment(moving, fixed, config.registration);
            testCase.verifyEqual(transformPointsForward(estimated, moving), fixed, 'AbsTol', 1e-10);
            testCase.verifyEqual(info.rotationDeterminant, 1, 'AbsTol', 1e-12);
        end
        function halfTurnDoesNotCreateReflection(testCase)
            stream = RandStream('mt19937ar', 'Seed', 11);
            fixed = rand(stream, 400, 3).*[6, 3, 0.7];
            known = rigidtform3d([0, 0, 180], [1, -4, 2]);
            moving = transformPointsInverse(known, fixed);
            config = layerview.defaultConfig();
            config.registration.scoreNeighbors = 1;
            estimated = layerview.initialAlignment(moving, fixed, config.registration);
            testCase.verifyEqual(transformPointsForward(estimated, moving), fixed, 'AbsTol', 1e-10);
            testCase.verifyEqual(det(estimated.R), 1, 'AbsTol', 1e-12);
        end
        function homographyPreservesHeight(testCase)
            points = [1, 2, 3; 4, 5, 6];
            h = [2, 0, 4; 0, 3, -1; 0, 0, 1];
            expected = [6, 5, 3; 12, 14, 6];
            testCase.verifyEqual(layerview.transformXY(points, h), expected, 'AbsTol', 1e-12);
        end
        function homographyRejectsPointsAtInfinity(testCase)
            h = [1, 0, 0; 0, 1, 0; 1, 0, -1];
            testCase.verifyError(@() layerview.transformXY([1, 2, 3], h), 'layerview:HomographyAtInfinity');
        end
    end
end
