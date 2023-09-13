clear; close all;

lines_clr = lines(5);
blue = lines_clr(1,:);
orange = lines_clr(2,:);
purple = [175, 89, 186]/255;
dark = [2, 49, 71]/255;
green = [33, 139, 59]/255;

variable_names = {
    'ISM_3_length';
    'ISM_3_rest_length';
    'ISM_3_force';
    'ISM_3_hill_model_comps';
    'ISM_3_excitation';
    'ISM_3_activation';
    'fol_04'
};

%% N and M only
path = GetFullPath('../../bundle_reduced_NM_3contraction');
for i = 1:length(variable_names)
    filepath = path + "/" + variable_names{i} + ".csv";
    if exist(filepath, 'file') == 2
        eval([variable_names{i}, ' = load(filepath);']);
    end
end
nFrame = size(ISM_3_length, 1);
times = (1:nFrame)/120;

%%% H: plot C2 top x, y, z
figure('Position', [200, 200, 300, 200], 'Color', 'w'); hold on;
[az, el, top_bot_eyenose] = readFollicleAzEl(path, 'eyenose');
plot(times, top_bot_eyenose{5}(:, 1)-top_bot_eyenose{5}(1, 1), '-', 'Color', purple);
plot(times, top_bot_eyenose{5}(:, 2)-top_bot_eyenose{5}(1, 2), '-', 'Color', orange);
plot(times, top_bot_eyenose{5}(:, 3)-top_bot_eyenose{5}(1, 3), '-', 'Color', green);
legend({'x', 'y', 'z'}, 'location', 'southeast');
ylabel('C2 follice top displacements')
xlabel('simulation time (s)')
xticks(0:0.5:times(end));
grid on
print('FigResult1H', '-dtiff', '-r300');

%%% I: plot C2 az and el
figure('Position', [200, 200, 300, 200], 'Color', 'w'); hold on;
yyaxis left
plot(times, az(:, 4+1), 'DisplayName', 'az');
ylim([30, 120]);
yticks(40:20:120)
ylabel('C2 azimuth angle \theta')
yyaxis right
plot(times, el(:, 4+1), 'DisplayName', 'el');
ylim([0, 20]);
yticks(0:5:20);
ylabel('C2 elevation angle \phi')
xticks(0:0.5:times(end));
xlabel('simulation time (s)')
legend('Location', 'northeast', 'Box', 'off')
print('FigResult1I', '-dtiff', '-r300');


%% PIP and PM only
path = GetFullPath('../../bundle_reduced_PIPPM_3contraction');
for i = 1:length(variable_names)
    filepath = path + "/" + variable_names{i} + ".csv";
    if exist(filepath, 'file') == 2
        eval([variable_names{i}, ' = load(filepath);']);
    end
end
nFrame = size(ISM_3_length, 1);
times = (1:nFrame)/120;

%%% K: plot C2 top x, y, z
figure('Position', [200, 200, 300, 200], 'Color', 'w'); hold on;
[az, el, top_bot_eyenose] = readFollicleAzEl(path, 'eyenose');
plot(times, top_bot_eyenose{5}(:, 1)-top_bot_eyenose{5}(1, 1), '-', 'Color', purple);
plot(times, top_bot_eyenose{5}(:, 2)-top_bot_eyenose{5}(1, 2), '-', 'Color', orange);
plot(times, top_bot_eyenose{5}(:, 3)-top_bot_eyenose{5}(1, 3), '-', 'Color', green);
legend({'x', 'y', 'z'}, 'location', 'southeast');
ylabel('C2 follice top displacements')
ylim([-1, 0.5])
xlabel('simulation time (s)')
xticks(0:0.5:times(end));
grid on
print('FigResult1K', '-dtiff', '-r300');

%%% L: plot C2 az and el
figure('Position', [200, 200, 300, 200], 'Color', 'w'); hold on;
yyaxis left
plot(times, az(:, 4+1), 'DisplayName', 'az');
ylim([30, 120]);
yticks(40:20:120)
ylabel('C2 azimuth angle \theta')
yyaxis right
plot(times, el(:, 4+1), 'DisplayName', 'el');
ylim([0, 20]);
yticks(0:5:20);
ylabel('C2 elevation angle \phi')
xticks(0:0.5:times(end));
xlabel('simulation time (s)')
legend('Location', 'northeast', 'Box', 'off')
print('FigResult1L', '-dtiff', '-r300');


