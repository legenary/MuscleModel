clear; close all;

fps = 120;

fiber = load('../bundle/fiber_info.csv');

figure('Color', 'w'); hold on;

plot(fiber(1,:), 'k');
plot(fiber(3,:), 'r');
plot(fiber(7,:), 'b');


