clear; close all;

lines_clr = lines(5);
blue = lines_clr(1,:);
orange = lines_clr(2,:);
purple = [175, 89, 186]/255;
dark = [2, 49, 71]/255;
green = [33, 139, 59]/255;
yellow = [255, 190, 13]/255;

variable_names = {
    'ISM_03_length';
    'ISM_03_rest_length';
    'ISM_03_force';
    'ISM_03_hill_model_comps';
    'ISM_03_excitation';
    'ISM_03_activation';
    'fol_04';
    'N_excitation';
    'N_activation'
};

path = GetFullPath('../../bundle_reduced');
for i = 1:length(variable_names)
    filepath = path + "/" + variable_names{i} + ".csv";
    if exist(filepath, 'file') == 2
        eval([variable_names{i}, ' = load(filepath);']);
    end
end
nFrame = size(ISM_03_length, 1);
times = (1:nFrame)/120;

figure('Color', 'w', 'Position', [200, 200, 900, 500]); hold on;
sgtitle('all muscles with phasing');
% first
subplot(411); hold on;
plot(times, ISM_03_length/ISM_03_length(1), '-', 'Color', dark, 'LineWidth', 2);
ylabel('C2 ISM length')
ylim([0.7, 1.1])
xlim([0,6]);
grid on
box on
xticks(0:0.5:times(end));
% second
subplot(412); hold on;
[az, el, top_bot_eyenose] = readFollicleAzEl(path, 'eyenose');
plot(times, top_bot_eyenose{5}(:, 1)-top_bot_eyenose{5}(1, 1), '-', 'Color', yellow);
plot(times, top_bot_eyenose{5}(:, 2)-top_bot_eyenose{5}(1, 2), '-', 'Color', purple, 'LineWidth', 1.4);
plot(times, top_bot_eyenose{5}(:, 3)-top_bot_eyenose{5}(1, 3), '-', 'Color', green);
legend({'x', 'y', 'z'}, 'location', 'southeast', 'Box', 'off');
xlim([0,6]);
ylim([-2.8, 0.9])
ylabel('C2 follice top XYZ')
xticks(0:0.5:times(end));
grid on
box on
% third
subplot(413); hold on;
plot(times, az(:, 4+1)-az(1, 4+1), 'DisplayName', 'az', 'LineWidth', 1.4);
ylabel('C2 azimuth angle \theta')
plot(times, el(:, 4+1)-el(1, 4+1), 'DisplayName', 'el');
xlim([0,6]);
ylim([-50, 50])
ylabel('C2 elevation angle \phi')
xticks(0:0.5:times(end));
grid on
box on
legend('Location', 'northeast', 'Box', 'off')
% fourth: activation
subplot(414); hold on;
plot((ISM_03_activation-0.5) + 1);
plot((N_activation-0.5) - 1);
% ylim([-0.2, 1.2])
xlim([0,720]);
xticks(0:60:600)
xticklabels(0:0.5:5)
ylabel('Different phases')
legend({'ISM protractors', 'Extrinsic retractors'})
grid on

print('FigResult2B', '-dtiff', '-r300');
