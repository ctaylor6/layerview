classdef TestSampling < matlab.unittest.TestCase
    methods (Test)
        function samplesStayInsideTriangle(testCase)
            vertices = [0, 0, 0; 2, 0, 0; 0, 1, 0];
            options = struct('seed', 17, 'pointsPerSquareMM', 10000, 'maxPoints', 1000);
            [points, info] = layerview.sampleMesh(vertices, [1, 2, 3], options);
            testCase.verifySize(points, [1000, 3]);
            testCase.verifyTrue(info.capped);
            testCase.verifyGreaterThanOrEqual(min(points(:, 1:2), [], 'all'), 0);
            testCase.verifyLessThanOrEqual(max(points(:, 1)/2+points(:, 2)), 1+1e-12);
            testCase.verifyEqual(points(:, 3), zeros(1000, 1));
        end
        function seededSamplingPreservesGlobalRNG(testCase)
            vertices = [0, 0, 0; 2, 0, 0; 0, 1, 0];
            options = struct('seed', 17, 'pointsPerSquareMM', 1000, 'maxPoints', 100);
            before = rng;
            p1 = layerview.sampleMesh(vertices, [1, 2, 3], options);
            p2 = layerview.sampleMesh(vertices, [1, 2, 3], options);
            testCase.verifyEqual(p1, p2);
            testCase.verifyEqual(rng, before);
        end
        function zeroAreaFacesAreSkipped(testCase)
            vertices = [0, 0, 0; 2, 0, 0; 0, 1, 0];
            options = struct('seed', 17, 'pointsPerSquareMM', 100, 'maxPoints', 1000);
            [~, info] = layerview.sampleMesh(vertices, [1, 1, 2; 1, 2, 3], options);
            testCase.verifyEqual(info.skippedDegenerateFaces, 1);
            testCase.verifyError(@() layerview.sampleMesh(vertices, [1, 1, 2], options), 'layerview:EmptyMesh');
        end
        function sampleCountsFollowTriangleArea(testCase)
            vertices = [0, 0, 0; 2, 0, 0; 0, 1, 0; 10, 0, 0; 16, 0, 0; 10, 1, 0];
            options = struct('seed', 1, 'pointsPerSquareMM', 10000, 'maxPoints', 400);
            points = layerview.sampleMesh(vertices, [1, 2, 3; 4, 5, 6], options);
            testCase.verifyEqual(sum(points(:, 1) < 5), 100);
            testCase.verifyEqual(sum(points(:, 1) > 5), 300);
        end
    end
end
