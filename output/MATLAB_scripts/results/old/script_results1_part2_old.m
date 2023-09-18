clear; close all;

lines_clr = lines(5);
blue = lines_clr(1,:);
orange = lines_clr(2,:);
purple = [175, 89, 186]/255;
dark = [2, 49, 71]/255;
green = [33, 139, 59]/255;
yellow = [255, 190, 13]/255;

path_list = {
    '../../bundle_reduced_ISM_3c',...
    '../../bundle_reduced_NM_3c',...
    '../../bundle_reduced_PIPPM_3c',...
    '../../bundle_reduced_all_3c_async',...
    '../../bundle_reduced_all_3c_sync'
};
title_list = {
    'ISM only',...
    'N/M only',...
    'PIP/PM only',...
    'All, out of sync',...
    'All, in sync'
};
figName_list = {
    'FigResult1D',...
    'FigResult1E',...
    'FigResult1F',...
    'FigResult1G',...
    'FigResult1H',...
};


%% 
for nFig = 1:5
    path = GetFullPath(path_list{nFig});
    for i = 1:length(variable_names)
        filepath = path + "/" + variable_names{i} + ".csv";
        if exist(filepath, 'file') == 2
            eval([variable_names{i}, ' = load(filepath);']);
        end
    end
    nFrame = size(ISM_03_length, 1);
    times = (1:nFrame)/120;

    figure('Color', 'w', 'Position', [200, 200, 300, 500]); hold on;
    sgtitle(title_list{nFig});
    % ISM length
    subplot(311); hold on;
    plot(times, ISM_03_length, '-', 'Color', dark);
    ylabel('C2 ISM length')
    ylim([3.5, 5.5])
    xlim([0,4]);
    grid on
    box on
    xticks(0:0.5:times(end));
    
    % displacements
    subplot(312); hold on;
    [az, el, top_bot_eyenose] = readFollicleAzEl(path, 'eyenose');
    plot(times, top_bot_eyenose{5}(:, 1)-top_bot_eyenose{5}(1, 1), '-', 'Color', yellow);
    plot(times, top_bot_eyenose{5}(:, 2)-top_bot_eyenose{5}(1, 2), '-', 'Color', purple);
    plot(times, top_bot_eyenose{5}(:, 3)-top_bot_eyenose{5}(1, 3), '-', 'Color', green);
    legend({'x', 'y', 'z'}, 'location', 'southeast', 'Box', 'off');
    xlim([0,4]);
    ylim([-2.8, 0.9])
    ylabel('C2 follice top XYZ')
    xticks(0:0.5:times(end));
    grid on
    box on
    % angles 
    subplot(313); hold on;
    plot(times, az(:, 4+1)-az(1, 4+1), 'DisplayName', 'az');
    ylabel('C2 azimuth angle \theta')
    plot(times, el(:, 4+1)-el(1, 4+1), 'DisplayName', 'el');
    xlim([0,4]);
    ylim([-50, 50])
    ylabel('C2 elevation angle \phi')
    xticks(0:0.5:times(end));
    grid on
    box on
    legend('Location', 'northeast', 'Box', 'off')
    
    print(figName_list{nFig}, '-dtiff', '-r300');
    
    
    daz = az(61:360, 4+1) - az(60:359, 4+1);
    del = el(61:360, 4+1) - el(60:359, 4+1);
    delta = smooth(del)./smooth(daz);
    delta = rmoutliers(delta);
    fprintf('del/daz = %.2f +/- %.2f\n', mean(delta), std(delta));
end

