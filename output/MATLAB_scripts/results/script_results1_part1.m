clear; close all;

lines_clr = lines(5);
blue = lines_clr(1,:);
orange = lines_clr(2,:);
purple = [175, 89, 186]/255;
dark = [2, 49, 71]/255;
green = [33, 139, 59]/255;

variable_names = {
    'ISM_03_length';
    'ISM_03_rest_length';
    'ISM_03_force';
    'ISM_03_hill_model_comps';
    'ISM_03_excitation';
    'ISM_03_activation';
    'fol_04'
};


%% ISM only
path = GetFullPath('../../bundle_reduced_ISM_3c');
for i = 1:length(variable_names)
    filepath = path + "/" + variable_names{i} + ".csv";
    if exist(filepath, 'file') == 2
        eval([variable_names{i}, ' = load(filepath);']);
    end
end
nFrame = size(ISM_03_length, 1);
times = (1:nFrame)/120;

%%% A: neural excitation
frame_stop = 360;
figure('Position', [200, 200, 150, 100], 'Color', 'w'); hold on;
plot(times(1:frame_stop), ISM_03_excitation(1:frame_stop), 'k-');
ylabel('neural excitation')
xticks(0:1:times(frame_stop))
xlabel('Simulation time (s)');
print('FigResult1A', '-dtiff', '-r300');

%%% B: muslce activation
figure('Position', [200, 200, 150, 100], 'Color', 'w'); hold on;
plot(times(1:frame_stop), ISM_03_activation(1:frame_stop), 'k-');
ylabel('muslce activation a');
xticks(0:1:times(frame_stop))
xlabel('Simulation time (s)');
print('FigResult1B', '-dtiff', '-r300');

%%% C: ISM muslce force vs length
figure('Position', [200, 200, 300, 200], 'Color', 'w'); hold on;

% left
yyaxis left
fPE = ISM_03_hill_model_comps(:, 1);
fL = ISM_03_hill_model_comps(:, 2);
fV = ISM_03_hill_model_comps(:, 3);
fAll = fPE + ISM_03_activation .* (fL .* fV);
plot(times, fPE, '--', 'Color', dark, 'DisplayName', 'fPE');
plot(times, fL, '--', 'Color', green, 'DisplayName', 'fL');
plot(times, fV, '--', 'Color', purple, 'DisplayName', 'fV');
plot(times, fAll, '-', 'Color', blue, 'DisplayName', 'fAll');
ylim([-0.1, 1.1])
yticks([0:0.25:1])
ylabel('force components')

yyaxis right
plot(times, ISM_03_length, '-', 'Color', orange, 'DisplayName', 'length');
ylabel('ISM Length'); ylim([3.3, 5.7])
title('ISM force components (left) and length (right)')
xticks(0:0.5:times(end));
xlabel('simulation time (s)')

grid on;
legend('Location', 'southeast', 'Box', 'off')

print('FigResult1C', '-dtiff', '-r300');

