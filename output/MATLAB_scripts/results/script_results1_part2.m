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
    load([path, '/bundle.mat']);
    nFrame = size(ISMs_length, 1);
    times = (1:nFrame)/120;

    figure('Color', 'w', 'Position', [200, 200, 300, 500]);
    % [ha, pos] = tight_subplot(Nh, Nw, gap, marg_h, marg_w)
    [ha, ~] = tight_subplot(4, 1, [0.03, 0.09], [0.15,0.07], [0.14,0.05]);
    sgtitle(title_list{nFig});
    % ISM length
    axes(ha(1)); hold on;
    plot(times, ISM_03.length, '-', 'Color', dark);
    ylabel('C2 ISM length')
    ylim([3.5, 5.5])
    xlim([0,4]);
    grid on
    box on
    xticks(0:0.5:4);
    xticklabels({});
    
    % displacements
    axes(ha(2)); hold on;
    plot(times, fols_top_bot_eyenose{5,1}(:, 1)-fols_top_bot_eyenose{5,1}(1, 1), '-', 'Color', yellow);
    plot(times, fols_top_bot_eyenose{5,1}(:, 2)-fols_top_bot_eyenose{5,1}(1, 2), '-', 'Color', purple);
    plot(times, fols_top_bot_eyenose{5,1}(:, 3)-fols_top_bot_eyenose{5,1}(1, 3), '-', 'Color', green);
    legend({'x', 'y', 'z'}, 'location', 'southeast', 'Box', 'off');
    xlim([0,4]);
    ylim([-2.8, 0.9])
    ylabel('C2 follice top XYZ')
    xticks(0:0.5:4);
    xticklabels({});
    grid on
    box on
    
    % angles 
    axes(ha(3)); hold on
    plot(times, fols_az_eyenose(:, 4+1)-fols_az_eyenose(1, 4+1), 'DisplayName', 'az');
    plot(times, fols_el_eyenose(:, 4+1)-fols_el_eyenose(1, 4+1), 'DisplayName', 'el');
    xlim([0,4]);
    ylim([-50, 50])
    xticks(0:0.5:4);
    ylabel('C2 orientationa')
    xlabel('Simulation time (s)')
    grid on
    box on
    legend('Location', 'northeast', 'Box', 'off')
    
    fitRes = fitlm(fols_az_eyenose(61:360, 4+1), fols_el_eyenose(60:359, 4+1));
    slope = fitRes.Coefficients.('Estimate')('x1');
    slopeStd = fitRes.Coefficients.('SE')('x1');
    text(1.5, -40,...
        sprintf('del/daz = %.2f +/- %.2f\n', slope, slopeStd))
    

    axes(ha(4)); hold on; 
    ax = gca;
    ax.Position(2) = ax.Position(2) - 0.06;
    varnames = {
        'ISM01'; 'ISM02'; 'ISM03'; 'ISM04'; 'ISM05'; 'ISM06'; 
        'ISM07'; 'ISM08'; 'ISM09'; 'ISM10'; 'ISM11'; 'ISM12'; 
        'N'; 'M'; 'PIP'; 'PM';
    };
    width = 0.17;
    xlim([0, length(varnames)+1]);
    plot(xlim, [1, 1], 'Color', [.7, .7, .7]);
    for i = 1:length(varnames)
        if i <= 12
            var = ISMs_percentContraction(:, i);
        else
            var = eval([varnames{i}, '_percentContraction']);
        end
        X = [i-width, i+width, i+width, i-width];
        Y = [min(var), min(var), max(var), max(var)];
        patch('XData', X, 'YData', Y, 'EdgeColor', 'none',...
            'FaceColor', dark);
    end
    xticks(1:length(varnames));
    xticklabels(varnames);
    xtickangle(90);
    ylabel('Muscle relative length')
    ylim([0.6, 1.5]);
    print(figName_list{nFig}, '-dtiff', '-r300');
end


