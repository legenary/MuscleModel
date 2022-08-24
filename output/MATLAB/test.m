ca;

data = az(:, 5);

figure; hold on;
% plot(data);
[tops, locs] = findpeaks(data, 'MinPeakProminence', fps/3);
% plot(locs, tops, 'bo');
[bots, locs] = findpeaks(-data, 'MinPeakProminence', fps/3);
% plot(locs, -bots, 'bo');

tops = tops(1:min([length(tops),length(bots)]), :);
bots = -bots(1:min([length(tops),length(bots)]), :);

diffs = tops-bots;
plot(diffs, 'k*');

[b, a] = findConvergence1d(diffs);
x = 1:0.5:20;
y = a./x + b;
plot(x, y, 'r-');



