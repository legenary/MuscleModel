close all;
figure('Position', [200, 200, 400, 150], 'Color', 'w'); hold on
fps = 120;
T = 5;


pos_60 = load('../test_60.csv');
plot(linspace(0, T, T*60), pos_60);

pos_120 = load('../test_120.csv');
plot(linspace(0, T, T*120), pos_120);

legend('60Hz', '120Hz');

ax = gca;
ax.YLim = [-0.6, 0];
hdl = hline(-0.5, 'k:');
box on;