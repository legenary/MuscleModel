clear; close all;

lines_clr = lines(5);
blue = lines_clr(1,:);
orange = lines_clr(2,:);
purple = [175, 89, 186]/255;
dark = [2, 49, 71]/255;
green = [33, 139, 59]/255;
yellow = [255, 190, 13]/255;

path = GetFullPath('../../bundle_full');

[az, el, top_bot] = readFollicleAzEl(path, 'eyenose');
az = az(120:end, :);
el = el(120:end, :);
Row = [1 1 1 1 1, 2 2 2 2 2, 3 3 3 3 3 3, 4 4 4 4 4 4 4 4, 5 5 5 5 5 5 5];
Col = [1 2 3 4 5, 1 2 3 4 5, 1 2 3 4 5 6, 1 2 3 4 5 6 7 8, 2 3 4 5 6 7 8];

colColors = lines(8);
rowColors = lines(5);

% figure; hold on;
% plot(ISM_12_length);
% xticks(0:60:720)

% figure A: el vs az
figure('Color', 'w'); hold on;
protracting_frames = [1:60; 121:180; 241:300; 361:420; 481:540];
retracting_frames = [61:120; 181:240; 301:360; 421:480; 541:600];
for r = 1:5
    idx = find(Row==r);
    for i = idx
        if i == 24 || i == 31
            continue;
        end
        for c = 1:5
            plot(az(retracting_frames(c,:), i), el(retracting_frames(c,:), i), 'Color', 1-(1-rowColors(Row(i), :))/5);
        end
    end
end
for r = 1:5
    idx = find(Row==r);
    slopeThisRow = nan(length(idx), 1);
    RsquaredThisRow = nan(length(idx), 1);
    n = 1;
    for i = idx
        if i == 24 || i == 31
            continue;
        end
        for c = 1:5
            plot(az(protracting_frames(c,:), i), el(protracting_frames(c,:), i), 'Color', rowColors(Row(i), :));
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
xlabel('follicle azimuth \theta');
ylabel('follicle elevation \phi');
grid on
axis equal
ylim([-30, 70]);
xlim([25, 135]);
ISM_Row = [1 1 1 1, 2 2 2 2 2, 3 3 3 3 3 3, 4 4 4 4 4 4 4 4, 5 5 5 5 5 5 5, 1 2 3 4];
ISM_Col = [2 3 4 5, 2 2 3 4 5, 2 2 3 4 5 6, 2 2 3 4 5 6 7 8, 2 3 4 5 6 7 8, 1 1 1 1];
ISM2Folidx = [2 3 4 5, 7 7 8 9 10, 12 12 13 14 15 16, 18 18 19 20 21 22 23 24, 25 25 26 27 28 29 30 31, 1 6 11 17];

% all ISM contraction % with time
nFrame = size(load([path, '/fol_00.csv']), 1);
ISMs_length = nan(34, nFrame);
for i = 1:34
    variable_name = sprintf('ISM_%02d_length', i-1);
    filepath = path + "/" + variable_name + ".csv";
    if exist(filepath, 'file') == 2
        ISMs_length(i, :) = load(filepath);
    end
end
times = (1:nFrame)/120;

[az, el, top_bot] = readFollicleAzEl(path, 'eyenose');

figure('Color', 'w'); hold on;
ISMs_percentContraction = nan(34, 120);
ISMs_az = nan(34, 120);
for i = 1:34
    if ~isnan(ISMs_length(i, 1))
        ISM_length = ISMs_length(i, :);
        % 5 contractions
        ISM_percentContraction = zeros(5, 120);
        ISM_az = zeros(5, 120);
        for c = 1:5
            startFrame = 120*c;
            endFrame = startFrame+119;
            restLength = ISM_length(1);
            ISM_percentContraction(c, :) = ISM_length(startFrame:endFrame)/restLength;
            ISM_az(c, :) = az(startFrame:endFrame, ISM2Folidx(i))';
        end
        ISM_percentContraction = mean(ISM_percentContraction, 1);
        ISMs_percentContraction(i, :) = ISM_percentContraction;
        ISM_az = mean(ISM_az, 1);
        ISMs_az(i, :) = ISM_az;
        lw = 0.5;
        if i == 13
            lw = 3;
        end
        plot(ISM_percentContraction, 'Color', colColors(ISM_Col(i), :),...
            'LineWidth', lw);
    end
end
ylim([0.7, 1.1]);
xticks(0:60:120);
xticklabels(0:0.5:1);
xlabel('time');
ylabel('ISM percent of rest length')
yticks(0.7:0.1:1.1);
yticklabels(70:10:110);
hline(1);

% all ISM protraction vs contraction 
% row 2, 3, 4, 5, col 2, 3, 4, 5, 6 (same with ARP)
figure('Color', 'w'); hold on;
for i = 1:34
    if any(ISM_Col(i) == [2, 3, 4, 5, 6]) && any(ISM_Row(i) == [2, 3, 4, 5])
        [~, idx_closest_to_100percent] = min((ISMs_percentContraction(i, 1:60)-1).^2);
        plot(-ISMs_percentContraction(i, 1:60), ISMs_az(ISM2Folidx(i), 1:60)-ISMs_az(ISM2Folidx(i), idx_closest_to_100percent),...
            'Color', colColors(ISM_Col(i), :));
    end
end
hline(0); vline(-1);
xlabel('ISM percent of rest length');
ylabel('Fol protraction angle \Delta\theta');


% all ISM motor angle vs contraction
ISMs_motor = nan(34, 120);
for i = 1:34
    if ~isnan(ISMs_length(i, 1))
        % 5 contractions
        ISM_motor = zeros(5, 120);
        for c = 1:5
            startFrame = 120*c;
            endFrame = startFrame+119;
            rest_dir = top_bot{ISM2Folidx(i), 1}(startFrame, :) - ...
                top_bot{ISM2Folidx(i), 2}(startFrame, :);
            for f = startFrame:endFrame
                this_dir = top_bot{ISM2Folidx(i), 1}(f, :) - ...
                    top_bot{ISM2Folidx(i), 2}(f, :);
                ISM_motor(c, f-startFrame+1) = atan2(norm(cross(rest_dir,this_dir)), dot(rest_dir,this_dir));
            end
        end
        ISM_motor = mean(ISM_motor, 1);
        ISMs_motor(i, :) = ISM_motor;
    end
end

% all ISM protraction vs contraction 
% row 2, 3, 4, 5, col 2, 3, 4, 5, 6 (same with ARP)
figure('Color', 'w'); hold on;
for i = 1:34
    if any(ISM_Col(i) == [2, 3, 4, 5, 6]) && any(ISM_Row(i) == [2, 3, 4, 5])
        [~, idx_closest_to_100percent] = min((ISMs_percentContraction(i, 1:60)-1).^2);
        plot(-ISMs_percentContraction(i, 1:60), ISMs_motor(ISM2Folidx(i), 1:60)-ISMs_motor(ISM2Folidx(i), idx_closest_to_100percent),...
            'Color', colColors(ISM_Col(i), :));
    end
end
hline(0); vline(-1);
xlabel('ISM percent of rest length');
ylabel('Fol motor angle \Delta\theta');
