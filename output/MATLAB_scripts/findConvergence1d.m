function [b, a] = findConvergence1d(trend)
%findConvergence1d calculates the convergence point of a series of data
%points by fitting them to an inverse proportional function. This function
%is not designed to be robust so please clean the data first.
%
%           [b, a] = findConvergence1d(trend) trend is a 1d vector
%                       fit to : y = a * 1/x + b

% Yifu
% 2022/08/15

if ~any(size(trend)==1)
    error('Wrong data size, need 1D');
end
trend = trend(:);

isIncresing = (trend(end) > trend(1));

if isIncresing
    ab = fminsearch(@(x) costFunc(x, trend), [-1, 0]);
else
    ab = fminsearch(@(x) costFunc(x, trend), [1, 0]);
end

a = ab(1);
b = ab(2);

    function J = costFunc(ab, trend)
        y = ab(1)./(1:length(trend)) + ab(2);
        J = sum((y - trend').^2);
    end

end