classdef TestCalibration < matlab.unittest.TestCase
    methods (Test)
        function integerInputRetainsFractionalCoordinates(testCase)
            points = int32([1, 3, 7]);
            actual = layerview.transformXY(points, diag([0.5, 0.5, 1]));
            testCase.verifyEqual(actual, [0.5, 1.5, 7]);
        end
        function fourCorrespondencesRecoverProjectiveMap(testCase)
            image = [10, 20; 500, 30; 480, 400; 25, 390];
            known = [0.03, 0.002, -1; 0.001, 0.04, 2; 0.0001, -0.00005, 1];
            expected = layerview.transformXY([image, zeros(4, 1)], known);
            [estimated, info] = layerview.estimateHomography(image, expected(:, 1:2));
            actual = layerview.transformXY([image, zeros(4, 1)], estimated);
            testCase.verifyEqual(actual, expected, 'AbsTol', 1e-10);
            testCase.verifyLessThan(info.reprojectionRMSE, 1e-10);
        end
        function overdeterminedCalibrationPredictsHeldOutPoints(testCase)
            [x, y] = meshgrid(0:80:640, 0:60:480);
            image = [x(:), y(:)];
            known = [0.02, -0.003, 2; 0.001, 0.03, -1; 0.00002, 0.00001, 1];
            expected = layerview.transformXY([image, zeros(size(image, 1), 1)], known);
            estimated = layerview.estimateHomography(image, expected(:, 1:2));
            heldOut = [17, 33, 0.8; 299, 201, 1.2; 601, 403, 2];
            testCase.verifyEqual(layerview.transformXY(heldOut, estimated), ...
                layerview.transformXY(heldOut, known), 'AbsTol', 1e-10);
        end
        function degenerateCorrespondencesFailClearly(testCase)
            points = [0, 0; 1, 0; 2, 0; 3, 0];
            testCase.verifyError(@() layerview.estimateHomography(points, points), 'layerview:DegenerateCalibration');
        end
    end
end
