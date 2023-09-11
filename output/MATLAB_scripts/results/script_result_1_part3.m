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

%%
path = GetFullPath('../../bundle_reduced_ISMNMPIPPM_3contraction_sync');
for i = 1:length(variable_names)
    filepath = path + "/" + variable_names{i} + ".csv";
    if exist(filepath, 'file') == 2
        eval([variable_names{i}, ' = load(filepath);']);
    end
end
nFrame = size(ISM_3_length, 1);
times = (1:nFrame)/120;

%%% M: ISM muslce force vs length
figure('Position', [200, 200, 300, 200], 'Color', 'w'); hold on;

% left
plot(times, ISM_3_length, '-', 'Color', orange);
ylabel('ISM Length'); ylim([3.3, 5.7])
xticks(0:0.5:times(end));
xlabel('simulation time (s)')
grid on;

print('FigResult1M', '-dtiff', '-r300');


%%% N: plot C2 top x, y, z
figure('Position', [200, 200, 300, 200], 'Color', 'w'); hold on;
[az, el, top_bot_eyenose] = readFollicleAzEl(path, 'eyenose');
plot(times, top_bot_eyenose{5}(:, 1)-top_bot_eyenose{5}(1, 1), '-', 'Color', purple);
plot(times, top_bot_eyenose{5}(:, 2)-top_bot_eyenose{5}(1, 2), '-', 'Color', orange);
plot(times, top_bot_eyenose{5}(:, 3)-top_bot_eyenose{5}(1, 3), '-', 'Color', green);
legend({'x', 'y', 'z'}, 'location', 'southeast');
ylabel('C2 follice top displacements')
ylim([-2.5, 0.5])
xlabel('simulation time (s)')
xticks(0:0.5:times(end));
grid on
print('FigResult1N', '-dtiff', '-r300');

%%% O: plot C2 az and el
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
print('FigResult1O', '-dtiff', '-r300');


%%
path = GetFullPath('../../bundle_reduced_ISMNMPIPPM_3contraction_async');
for i = 1:length(variable_names)
    filepath = path + "/" + variable_names{i} + ".csv";
    if exist(filepath, 'file') == 2
        eval([variable_names{i}, ' = load(filepath);']);
    end
end
nFrame = size(ISM_3_length, 1);
times = (1:nFrame)/120;

%%% P: ISM muslce force vs length
figure('Position', [200, 200, 300, 200], 'Color', 'w'); hold on;

% left
plot(times, ISM_3_length, '-', 'Color', orange);
ylabel('ISM Length'); ylim([3.3, 5.7])
xticks(0:0.5:times(end));
xlabel('simulation time (s)')
grid on;

print('FigResult1P', '-dtiff', '-r300');


%%% Q: plot C2 top x, y, z
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
print('FigResult1Q', '-dtiff', '-r300');

%%% R: plot C2 az and el
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
print('FigResult1R', '-dtiff', '-r300');

% for 3 seconds, 360 frames
daz = az(2:360, 4+1) - az(1:359, 4+1);
del = el(2:360, 4+1) - el(1:359, 4+1);
delta = smooth(del)./smooth(daz);
delta = rmoutliers(delta);
fprintf('1 cycle, 120 frames, elevation per degree of azimuth: %.2f +/- %.2f\n',...
    mean(delta), std(delta));
fprintf('using MALTAB smooth and rmoutliers functions\n')

delta_ISMonly = load('deldaz_ISMonly').delta;
[h, p] = ttest2(delta,  delta_ISMonly, ...
    'alpha', 0.05, 'Vartype', 'equal');  



