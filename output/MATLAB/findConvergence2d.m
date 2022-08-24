function [b, a] = findConvergence2d(trend)
%findConvergence2d calculates the convergence point of a series of data
%points by fitting them to an inverse proportional function. This function
%is not designed to be robust so please clean the data first.
%
%           [b, a] = findConvergence2d(trend) trend is a 2-column data
%                       fit to : y = a * 1/x + b

% Yifu
% 2022/08/15

if size(trend, 1) == 2
    trend = trend';
end
if size(trend, 2) ~= 2
    error('Wrong data size, need 2D');
end

isIncresing = (trend(end, 2) > trend(1, 2));

if isIncresing
    ab = fminsearch(@(x) costFunc(x, trend), [-1, 0]);
else
    ab = fminsearch(@(x) costFunc(x, trend), [1, 0]);
end

a = ab(1);
b = ab(2);

    function J = costFunc(ab, trend)
        y = ab(1)./trend(:,1) + ab(2);
        J = sum((y - trend(:,2)).^2);
    end

end