function transform = composeTransforms(first, second)
%COMPOSETRANSFORMS Apply FIRST, then SECOND (premultiply convention).
transform = rigidtform3d(second.A*first.A);
end
