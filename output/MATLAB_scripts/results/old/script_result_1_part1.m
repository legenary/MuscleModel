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

%% 1.5 cycle ISM only
% path = GetFullPath('../../bundle_reduced_ISM_1.5contraction');
% for i = 1:length(variable_names)
%     filepath = path + "/" + variable_names{i} + ".csv";
%     if exist(filepath, 'file') == 2
%         eval([variable_names{i}, ' = load(filepath);']);
% %         figure; hold on;
% %         eval(['plot(', variable_names{i}, ');']);
% %         title(variable_names{i}, 'Interpreter', 'none');
%     end
% end
% nFrame = size(ISM_3_length, 1);
% times = (1:nFrame)/120;
% 
% 
% %%% E: ISM muslce force vs length
% figure('Position', [200, 200, 300, 200], 'Color', 'w'); hold on;
% 
% % left
% yyaxis left
% fPE = ISM_3_hill_model_comps(:, 1);
% fL = ISM_3_hill_model_comps(:, 2);
% fV = ISM_3_hill_model_comps(:, 3);
% fAll = fPE + ISM_3_activation .* (fL .* fV);
% plot(times, fPE, '--', 'Color', dark, 'DisplayName', 'fPE');
% plot(times, fL, '--', 'Color', green, 'DisplayName', 'fL');
% plot(times, fV, '--', 'Color', purple, 'DisplayName', 'fV');
% plot(times, fAll, '-', 'Color', blue, 'DisplayName', 'fAll');
% ylim([-0.1, 1.1])
% yticks([0:0.25:1])
% ylabel('force components')
% 
% yyaxis right
% plot(times, ISM_3_length, '-', 'Color', orange, 'DisplayName', 'length');
% ylabel('ISM Length'); ylim([3.3, 5.7])
% title('ISM force components (left) and length (right)')
% xticks(0:0.5:times(end));
% xlabel('simulation time (s)')
% 
% grid on;
% legend('Location', 'southeast', 'Box', 'off')
% 
% print('FigResult1E', '-dtiff', '-r300');
% 
% %%% F: ISM: C2 az vs el
% figure('Position', [200, 200, 300, 200], 'Color', 'w'); hold on;
% 
% [top, bot] = readFollicleTopBot(path);
% [az, el] = readFollicleAzEl(path, 'eyenose');
% % plot azimuthal angle of follicle C2: #4 (0-indexing)
% yyaxis left
% plot(times, az(:, 4+1), 'DisplayName', 'az');
% ylim([30, 120]);
% yticks(40:20:120)
% ylabel('C2 azimuth angle \theta')
% yyaxis right
% plot(times, el(:, 4+1), 'DisplayName', 'el');
% ylim([0, 20]);
% yticks(0:5:20);
% ylabel('C2 elevation angle \phi')
% 
% xticks(0:0.5:times(end));
% xlabel('simulation time (s)')
% legend('Location', 'southeast', 'Box', 'off')
% 
% title('C2 follicle azimuthal angle change');
% print('FigResult1F', '-dtiff', '-r300');

%% 1 cycle ISM only
path = GetFullPath('../../bundle_reduced_ISM_1contraction');
for i = 1:length(variable_names)
    filepath = path + "/" + variable_names{i} + ".csv";
    if exist(filepath, 'file') == 2
        eval([variable_names{i}, ' = load(filepath);']);
    end
end
nFrame = size(ISM_3_length, 1);
times = (1:nFrame)/120;

%%% A: neural excitation
frame_stop = 180;
figure('Position', [200, 200, 150, 100], 'Color', 'w'); hold on;
plot(times(1:frame_stop), ISM_3_excitation(1:frame_stop), 'k-');
ylabel('neural excitation')
xticks(0:1:times(frame_stop))
xlabel('Simulation time (s)');
print('FigResult1A', '-dtiff', '-r300');

%%% B: muslce activation
figure('Position', [200, 200, 150, 100], 'Color', 'w'); hold on;
plot(times(1:frame_stop), ISM_3_activation(1:frame_stop), 'k-');
ylabel('muslce activation a');
xticks(0:1:times(frame_stop))
xlabel('Simulation time (s)');
print('FigResult1B', '-dtiff', '-r300');

%%% C: ISM muslce force vs length
figure('Position', [200, 200, 300, 200], 'Color', 'w'); hold on;

% left
yyaxis left
fPE = ISM_3_hill_model_comps(:, 1);
fL = ISM_3_hill_model_comps(:, 2);
fV = ISM_3_hill_model_comps(:, 3);
fAll = fPE + ISM_3_activation .* (fL .* fV);
plot(times, fPE, '--', 'Color', dark, 'DisplayName', 'fPE');
plot(times, fL, '--', 'Color', green, 'DisplayName', 'fL');
plot(times, fV, '--', 'Color', purple, 'DisplayName', 'fV');
plot(times, fAll, '-', 'Color', blue, 'DisplayName', 'fAll');
ylim([-0.1, 1.1])
yticks([0:0.25:1])
ylabel('force components')

yyaxis right
plot(times, ISM_3_length, '-', 'Color', orange, 'DisplayName', 'length');
ylabel('ISM Length'); ylim([3.3, 5.7])
title('ISM force components (left) and length (right)')
xticks(0:0.5:times(end));
xlabel('simulation time (s)')

grid on;
legend('Location', 'southeast', 'Box', 'off')

print('FigResult1C', '-dtiff', '-r300');

%%% D: ISM: C2 az vs el
figure('Position', [200, 200, 300, 200], 'Color', 'w'); hold on;

[top, bot] = readFollicleTopBot(path);
[az, el] = readFollicleAzEl(path, 'eyenose');
% plot azimuthal angle of follicle C2: #4 (0-indexing)
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
legend('Location', 'southeast', 'Box', 'off')
title('C2 follicle azimuthal angle change');



print('FigResult1D', '-dtiff', '-r300');

% for 1 second, 120 frames
daz = az(2:120, 4+1) - az(1:119, 4+1);
del = el(2:120, 4+1) - el(1:119, 4+1);
delta = smooth(del)./smooth(daz);
delta = rmoutliers(delta);
fprintf('1 cycle, 120 frames, elevation per degree of azimuth: %.2f +/- %.2f\n',...
    mean(delta), std(delta));
fprintf('using MALTAB smooth and rmoutliers functions\n')

save('deldaz_ISMonly.mat', 'delta');
% 
% daz = az(1:120, 4+1) - az(1, 4+1);
% del = el(1:120, 4+1) - el(1, 4+1);
% delta = rmoutliers(del./daz);
