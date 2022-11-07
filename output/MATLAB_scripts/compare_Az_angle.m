clear; close all;

T = 5;
% FPS = [90, 100, 110, 120];
% StepPerFrame = [500];
FPS = [120];
StepPerFrame = [60, 80, 100, 300, 500, 700];

Row = [1,1,1,1,1, 2,2,2,2,2, 3,3,3,3,3,3,3, 4,4,4,4,4,4,4, 5,5,5,5,5,5];
Col = [1,2,3,4,5, 1,2,3,4,5, 1,2,3,4,5,6,7, 1,2,3,4,5,6,7, 1,2,3,4,5,6];

figure('Position', [200, 200, 600, 300], 'Color', 'w'); hold on;
color = turbo(7);

n = 1;
for fps = FPS
    for stepPerFrame = StepPerFrame
        
        % determine time step, usually stepTotal = T*step - 1
        path = sprintf('../bundle_%dHz_%d', fps, stepPerFrame);
        stepTotal = size(load([path, '/fol_00.csv']), 1);

        fol_traj = zeros(31, stepTotal, 6);
        for fol = 1:31
            filename = sprintf('%s/fol_%02d.csv', path, fol-1);
            fol_traj(fol, :, :) = load(filename);
        end

        [az, el] = getAzElEyenose(fol_traj);

        hdl(n) = plot(az(:, 1), 'Color', color(n, :));
        
        n = n + 1;
    end
end
box on;
legend(hdl, arrayfun(@(x) [num2str(x)], StepPerFrame, 'uni', 0),...
    'Location', 'southeast');

% plot a vertical line every 1 second
loc = fps;
yl = ylim;
while(loc < (stepTotal+fps))
    h = vline(loc, 'k:');
    h.HandleVisibility = 'off';
    text(loc, yl(1)+3, sprintf('%ds', loc/fps));
    loc = loc + fps;
end


