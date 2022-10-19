clear; close all;

fps = 120;

load('fiber_length_120_100.csv');
dt = fiber_length_120_100;
steps = size(dt, 2);

figure('Color', 'w'); hold on;
plot(dt(1,:), 'r-');
plot(dt(2,:), 'k-');