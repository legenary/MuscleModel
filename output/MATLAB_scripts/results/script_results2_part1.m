clear; close all;

lines_clr = lines(5);
blue = lines_clr(1,:);
orange = lines_clr(2,:);
purple = [175, 89, 186]/255;
dark = [2, 49, 71]/255;
green = [33, 139, 59]/255;
yellow = [255, 190, 13]/255;

path = GetFullPath('../../bundle_reduced_all_20c_phasing_baseline');
load([path, '/bundle.mat']);
nFrame = size(ISM_03.length, 1);
times = (1:nFrame)/120;

endTime = 20;

figure('Color', 'w', 'Position', [200, 200, 400, 500]); hold on;
sgtitle('all muscles with phasing');
% first
subplot(411); hold on;
plot(times, ISM_03.length/ISM_03.length(1), '-', 'Color', dark, 'LineWidth', 2);
ylabel('C2 ISM length')
ylim([0.7, 1.1])
xlim([1,endTime]);
grid on
box on
xticks(0:0.5:4);
% second
subplot(412); hold on;

plot(times, fols_top_bot_eyenose{5,1}(:, 1)-fols_top_bot_eyenose{5,1}(1, 1), '-', 'Color', yellow);
plot(times, fols_top_bot_eyenose{5,1}(:, 2)-fols_top_bot_eyenose{5,1}(1, 2), '-', 'Color', purple, 'LineWidth', 1.4);
plot(times, fols_top_bot_eyenose{5,1}(:, 3)-fols_top_bot_eyenose{5,1}(1, 3), '-', 'Color', green);
legend({'x', 'y', 'z'}, 'location', 'southeast', 'Box', 'off');
xlim([1,endTime]);
ylim([-2.8, 0.9])
ylabel('C2 follice top XYZ')
xticks(0:0.5:4);
grid on
box on
% third
subplot(413); hold on;
plot(times, smooth(fols_az_eyenose(:, 4+1)-fols_az_eyenose(1, 4+1)), 'DisplayName', 'az', 'LineWidth', 1.4);
ylabel('C2 azimuth angle \theta')
plot(times, smooth(fols_el_eyenose(:, 4+1)-fols_el_eyenose(1, 4+1)), 'DisplayName', 'el');
xlim([1,endTime]);
ylim([-50, 50])
ylabel('C2 elevation angle \phi')
xticks(0:0.5:4);
grid on
box on
legend('Location', 'northeast', 'Box', 'off', 'location', 'southeast')
% fourth: activation
subplot(414); hold on;
plot(times, (ISM_03.activation-0.5) + 1);
plot(times, (N_activation-0.5) - 1);
% ylim([-0.2, 1.2])
xlim([1,endTime])
xticks(0:60:600)
xticklabels(0:0.5:5)
xticks(0:0.5:4);
ylabel('Activation per phase')
legend({'ISM protractors', 'Extrinsic retractors'})
grid on

print('FigResult2B', '-dtiff', '-r300');

% 

fitRes = fitlm(fols_az_eyenose(121:479, 4+1), fols_el_eyenose(121:479, 4+1));
slope = fitRes.Coefficients.('Estimate')('x1');
slopeStd = fitRes.Coefficients.('SE')('x1');
fprintf('del/daz = %.2f +/- %.2f\n', slope, slopeStd)