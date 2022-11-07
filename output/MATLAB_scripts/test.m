close all;

pos_60 = load('../test_60.csv');
pos_120 = load('../test_120.csv');

fps = 60;
T = 5;

figure('Position', [200, 200, 400, 150], 'Color', 'w'); hold on
plot(linspace(0, T, T*60), pos_60);
plot(linspace(0, T, T*120), pos_120);
legend('60Hz', '120Hz');
ax = gca;
ax.YTick = -0.75:0.25:0;
% ax.YLim = [-0.6, 0];
hdl = hline(-0.5, 'k:');
hdl.HandleVisibility = 'off';
box on;