clear; close all;

path = GetFullPath('../bundle_full');
variable_names = {
    % ISM_3 for reduced size
    'ISM_12_length';
    'ISM_12_rest_length';
    'ISM_12_force';
    'ISM_12_hill_model_comps';
    'ISM_12_excitation';
    'ISM_12_activation';
%     'fol_04'
};


for i = 1:length(variable_names)
    filepath = path + "/" + variable_names{i} + ".csv";
    if exist(filepath, 'file') == 2
        eval([variable_names{i}, ' = load(filepath);']);
%         figure; hold on;
%         eval(['plot(', variable_names{i}, ');']);
%         title(variable_names{i}, 'Interpreter', 'none');
    end
end
nFrame = size(ISM_12_length, 1);

figure; hold on;
yyaxis left
plot(ISM_12_hill_model_comps);
plot((ISM_12_hill_model_comps(:,1) + ISM_12_activation .*...
    ISM_12_hill_model_comps(:,2) .* ISM_12_hill_model_comps(:,2)))
ylabel('hill_model_comps');
ylim([-0.1, 1.1])
yyaxis right
plot(ISM_12_length);
ylabel('Length'); 
ylim([3.3, 5.7])
title('intrinsic muslce force components (left) and muscle length (right)')
legend({'fPE','fL','fV','all','Length'}, 'Location', 'southeast')
grid on;
xticks(0:60:nFrame)

[top, bot] = readFollicleTopBot(path);
[az, el, top_bot_eyenose] = readFollicleAzEl(path, 'eyenose');

figure; hold on;
clr = lines(3);
plot(top_bot_eyenose{5}(:, 1)-top_bot_eyenose{5}(1, 1), '-', 'Color', clr(1, :));
plot(top_bot_eyenose{5}(:, 2)-top_bot_eyenose{5}(1, 2), '-', 'Color', clr(2, :));
plot(top_bot_eyenose{5}(:, 3)-top_bot_eyenose{5}(1, 3), '-', 'Color', clr(3, :));
legend({'x', 'y', 'z'});
% title('fol04 (C2) displacement (y coordinate)')


figure; hold on;
% plot azimuthal angle of follicle #4 (0-indexing)
plot(az(:, 12+1));
plot(el(:, 12+1));
title('C2 follicle azimuthal angle change');
