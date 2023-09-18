clear; close all;

lines_clr = lines(5);
blue = lines_clr(1,:);
orange = lines_clr(2,:);
purple = [175, 89, 186]/255;
dark = [2, 49, 71]/255;
green = [33, 139, 59]/255;


%% ISM only
path = GetFullPath('../../bundle_reduced');

load([path, '/bundle.mat']);

nFrame = size(ISMs_length, 1);
times = (1:nFrame)/120;

%%% A: neural excitation
frame_stop = 360;
figure('Position', [200, 200, 150, 100], 'Color', 'w'); hold on;
plot(times(1:frame_stop), ISM_03.excitation(1:frame_stop), 'k-');
ylabel('neural excitation')
xticks(0:1:times(frame_stop))
xlabel('Simulation time (s)');
print('FigResult1A', '-dtiff', '-r300');

%%% B: muslce activation
figure('Position', [200, 200, 150, 100], 'Color', 'w'); hold on;
plot(times(1:frame_stop), ISM_03.activation(1:frame_stop), 'k-');
ylabel('muslce activation a');
xticks(0:1:times(frame_stop))
xlabel('Simulation time (s)');
print('FigResult1B', '-dtiff', '-r300');

%%% C: ISM muslce force vs length
figure('Position', [200, 200, 300, 200], 'Color', 'w'); hold on;

% left
yyaxis left
fPE = ISM_03.hill_model_components.fPE;
fL = ISM_03.hill_model_components.fL;
fV = ISM_03.hill_model_components.fV;
fAll = fPE + ISM_03.activation .* (fL .* fV);
plot(times, fPE, '--', 'Color', dark, 'DisplayName', 'fPE');
plot(times, fL, '--', 'Color', green, 'DisplayName', 'fL');
plot(times, fV, '--', 'Color', purple, 'DisplayName', 'fV');
plot(times, fAll, '-', 'Color', orange, 'DisplayName', 'fAll');
ylim([-0.1, 1.1])
yticks([0:0.25:1])
ylabel('force components')
set(gca,{'YColor'},{orange});

yyaxis right
plot(times, ISM_03.length, '-', 'Color', dark, 'DisplayName', 'length');
ylabel('ISM Length'); ylim([3.3, 5.7])
title('ISM force components (left) and length (right)')
xticks(0:0.5:times(end));
xlabel('simulation time (s)')
set(gca,{'YColor'},{dark});

grid on;
legend('Location', 'southeast', 'Box', 'off')

print('FigResult1C', '-dtiff', '-r300');

