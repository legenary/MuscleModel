clear; close all;

fps = 120;

fl_120_100 = load('../bundle/fiber_length.csv');
fl_120_300 = load('../bundle_120Hz_300/fiber_length.csv');
fl_120_500 = load('../bundle_120Hz_500/fiber_length.csv');
fl_120_700 = load('../bundle_120Hz_700/fiber_length.csv');

steps = size(fl_120_100, 2);

color = lines(5);
figure('Color', 'w'); hold on;
h0 = plot(fl_120_100(1,:), 'k-');

h1 = plot(fl_120_100(2,:), '-', 'Color', color(1, :));
h2 = plot(fl_120_300(2,:), '-', 'Color', color(2, :));
h3 = plot(fl_120_500(2,:), '-', 'Color', color(3, :));
h4 = plot(fl_120_700(2,:), '-', 'Color', color(4, :));

legend([h0, h1, h2, h3, h4], {'Intruction', '120Hz 100', '120Hz 300', '120Hz 500', '120Hz 700'})