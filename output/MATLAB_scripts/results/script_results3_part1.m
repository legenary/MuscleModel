clear; close all;

lines_clr = lines(5);
blue = lines_clr(1,:);
orange = lines_clr(2,:);
purple = [175, 89, 186]/255;
dark = [2, 49, 71]/255;
green = [33, 139, 59]/255;
yellow = [255, 190, 13]/255;

variable_names = {
    'ISM_12_length';
    'ISM_12_rest_length';
    'ISM_12_force';
    'ISM_12_hill_model_comps';
    'ISM_12_excitation';
    'ISM_12_activation';
};

path = GetFullPath('../../bundle_full');
for i = 1:length(variable_names)
    filepath = path + "/" + variable_names{i} + ".csv";
    if exist(filepath, 'file') == 2
        eval([variable_names{i}, ' = load(filepath);']);
    end
end
nFrame = size(ISM_12_length, 1);
times = (1:nFrame)/120;

[az, el, top_bot] = readFollicleAzEl(path, 'eyenose');
az = az(121:end, :);
el = el(121:end, :);
Row = [1 1 1 1 1, 2 2 2 2 2, 3 3 3 3 3 3, 4 4 4 4 4 4 4 4, 5 5 5 5 5 5 5];
Col = [1 2 3 4 5, 1 2 3 4 5, 1 2 3 4 5 6, 1 2 3 4 5 6 7 8, 2 3 4 5 6 7 8];

colColors = lines(8);
rowColors = lines(5);

figure; hold on;
plot(ISM_12_length);
xticks(0:60:720)


figure; hold on;
for i = 1:31
    if i == 24 || i == 31
        continue;
    end
%     plot(az(:, i), 'Color', colColors(Col(i), :));
%     plot(el(:, i), 'Color', rowColors(Row(i), :));
    plot(az(:, i), el(:, i), 'Color', rowColors(Row(i), :));
%     plot(az(:, i), el(:, i), 'Color', colColors(Col(i), :));

end

%deldaz per row
for r = 1:5
    idx = find(Row==r);
    slopeThisRow = nan(length(idx), 1);
    RsquaredThisRow = nan(length(idx), 1);
    n = 1;
    for i = idx
        if i == 24 || i == 31
            continue;
        end
        fitRes = fitlm(az(:,i), el(:,i));
        slopeThisRow(n) = fitRes.Coefficients.('Estimate')('x1');
        RsquaredThisRow(n) = fitRes.Rsquared.Ordinary;
        n = n + 1;
    end
    slopeThisRowMean = nanmean(slopeThisRow);
    slopeThisRowStd = nanstd(slopeThisRow);
    RsquaredThisRowMean = nanmean(RsquaredThisRow);

    fprintf('row%d: del/daz = %.2f +/- %.2f (R2 = %.2f) \n', r,...
        slopeThisRowMean, slopeThisRowStd, RsquaredThisRowMean);
end



