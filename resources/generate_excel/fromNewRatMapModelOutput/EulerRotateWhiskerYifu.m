function whisker_rotated = EulerRotateWhiskerYifu(theta, phi, zeta, whisker2D)


if size(whisker2D, 2) ~= 2
    error('Input whisker size should be n-by-2.');
end
nPts = size(whisker2D, 1);
whiskerStart = [
    zeros(nPts,1),
    -whisker2D(:, 1),
    -whisker2D(:, 2)
];


whisker_rotated = rotz(theta, 'deg')*...
             rotx(phi, 'deg')'*...
             roty(zeta, 'deg')*...
             whiskerStart;


end