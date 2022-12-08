clear;
close all;

%% Imporant info:
% Follicle position/orientation in the simulation are exported from ARP
% project, and therefore in mystacial pad reference frame. The follicles
% need to be successively transformed into 
%       (1) the avergae row plane, 
%       (2) adjusted eye-nose plane (Knutsen et al 2008) 
% for data analysis.

T = 5;
fps = 120;
stepPerFrame = 300;
queryRate = 60;
dampingRatio = 5;

% determine time step, usually stepTotal = T*step - 1
% path = sprintf('../bundle_%dHz_%d', fps, stepPerFrame);
path = sprintf('../bundle_%dHz_%dis_%dquery_%szeta', fps, stepPerFrame, queryRate, num2str(dampingRatio));
stepTotal = size(load([path, '/fol_00.csv']), 1);


fol_traj = zeros(31, stepTotal, 6);
for fol = 1:31
    filename = sprintf('%s/fol_%02d.csv', path, fol-1);
    fol_traj(fol, :, :) = load(filename);

end

[az, el] = getAzElEyenose(fol_traj);

%% plotting
Row = [1,1,1,1,1, 2,2,2,2,2, 3,3,3,3,3,3,3, 4,4,4,4,4,4,4, 5,5,5,5,5,5];
Col = [1,2,3,4,5, 1,2,3,4,5, 1,2,3,4,5,6,7, 1,2,3,4,5,6,7, 1,2,3,4,5,6];

% % 1. elevation angles
% figure('Color', 'w'); hold on;
% color = lines(5);
% for r = 1:5
%     plot(el(:, Row==r), 'Color', color(r,:));
% end
% xlabel('frames');
% ylabel('angles');
% title('Elevation angle by row');
% 
% % plot a vertical line every 1 second
% loc = fps;
% yl = ylim;
% while(loc < (stepTotal+fps))
%     vline(loc, 'k:');
%     text(loc, yl(1)+3, sprintf('%ds', loc/fps));
%     loc = loc + fps;
%     
% end

% 2. azimuth angles
% az = smoothdata(az, 'gaussian', 20);
% el = smoothdata(el, 'gaussian', 20);
figure('Position', [200, 200, 400, 250], 'Color', 'w'); hold on;
color = lines(5);
for r = 1:5
    plot(az(:, Row==r), 'Color', color(r,:));
%     plot(az(:, 1), 'Color', [0.9290 0.6940 0.1250]);
end
title({'Azimuthal angle by row', sprintf('%dHz, %d steps', fps, stepPerFrame)});
ylim([-60, 60])

% plot a vertical line every 1 second
loc = fps;
yl = ylim;
while(loc < (stepTotal+fps))
    vline(loc, 'k:');
    text(loc, yl(1)+3, sprintf('%ds', loc/fps));
    loc = loc + fps;
end


% Calculate del/daz 

% figure('Color', 'w'); hold on;
% frames = 482:541;
% % frames = 152:176;
% % frames = 202:226;
% fprintf("Frame: %d to %d\n", frames(1), frames(end));
% for r = 1:5
%     daz = az(frames(2:end), Row==r) - az(frames(1:end-1), Row==r);
%     del = el(frames(2:end), Row==r) - el(frames(1:end-1), Row==r);
%     daz = smoothdata(daz);
%     del = smoothdata(del);
%     delta = del./daz;
%     
%     plot(daz(:), del(:), 'o', 'Color', color(r,:));
%     
%     fprintf("Row %d: del/daz = %.2f +/- %.2f\n", ...
%         r, mean(delta(:)), std(delta(:)));
% 
% end





