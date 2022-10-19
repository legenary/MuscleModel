clear;
close all;

%% Imporant info:
% Follicle position/orientation in the simulation are exported from ARP
% project, and therefore in mystacial pad reference frame. The follicles
% need to be *successively* transformed into 
%       (1) the avergae row plane, 
%       (2) adjusted eye-nose plane (Knutsen et al 2008) 
% for data analysis.

%determine time step
steps = size(load("../fol_pos/fol_00.csv"), 1);
fps = 120;

fol_traj = zeros(31, steps, 6);
for fol = 1:31
    filename = sprintf("../fol_pos/fol_%02d.csv", fol-1);
    fol_traj(fol, :, :) = load(filename);

end

%% First, adjust to average row plane
PLOTTING = 0;
R = [0.9092,0.3570,0.2142;
    -0.3598,0.9326,-0.0270;
    -0.2094,-0.0525,0.9764];

if PLOTTING, figure('Color', 'w'); hold on; end

top_bot = cell(31, 2);

for i = 1:31
    top_bot{i, 1} = squeeze(fol_traj(i, :, 1:3))*rotz(pi/2)*roty(-pi/2);
    top_bot{i, 2} = squeeze(fol_traj(i, :, 4:6))*rotz(pi/2)*roty(-pi/2);
    top_bot{i, 1}(:,3) = -top_bot{i, 1}(:,3);
    top_bot{i, 2}(:,3) = -top_bot{i, 2}(:,3);
    if PLOTTING 
        plot3d(top_bot{i, 1}, 'ro');
        plot3d(top_bot{i, 1}*R, 'bo');
        plot3d(top_bot{i, 2}*R, 'b-');
    end
end
if PLOTTING 
    axis equal
    xlabel('x')
    ylabel('y')
end

top_bot_avgrow = cellfun(@(x) x*R, top_bot, 'uni', 0);

% Generate az and el angle
az_avgrow = zeros(steps, 31);
el_avgrow = zeros(steps, 31);
for i = 1:31
    dir = top_bot_avgrow{i, 1} - top_bot_avgrow{i, 2};
    [th, phi, ~] = cart2sph(dir(:,1), dir(:,2), dir(:,3));
    
    az_avgrow(:, i) = rad2deg(th);
    el_avgrow(:, i) = rad2deg(phi);
    
end

%% Then, adjust to eye-nose plane (Knutsen et al 2008) from avg row plane
% calculate eye-nose plane
eyes = [-9.721020847417353,-19.067448129825216,3.891614295391962;
    -6.268610691514697,-13.559304383739748,5.840277177970524;
    10.292905691029162,-19.444559678246000,3.425704247361483;
    7.208632295799561,-14.387253506531858,5.566668713822409];
noses = [-1.928577625443136,5.716489592791897,1.605123645981586;
    -0.734239178172683,7.323191087283126,0.977908851272219;
    2.622476944320339,5.453629555452386,1.748427308939650;
    1.341145614798153,7.520348466123311,1.065171840940149];
connect = mean(noses) - mean(eyes);
% angle is from average row plane to eye-nose plane:
angle = atand(connect(3)/connect(2));

top_bot_eyenose = cellfun(@(x) x*rotx(angle, 'deg'), top_bot_avgrow, 'uni', 0);

az_eyenose = zeros(steps, 31);
el_eyenose = zeros(steps, 31);
for i = 1:31
    dir = top_bot_eyenose{i, 1} - top_bot_eyenose{i, 2};
    [th, phi, ~] = cart2sph(dir(:,1), dir(:,2), dir(:,3));
    az_eyenose(:, i) = rad2deg(th);
    el_eyenose(:, i) = rad2deg(phi);
end

%% plotting
% choose what to plot here
el = el_eyenose;
az = az_eyenose;

Row = [1,1,1,1,1, 2,2,2,2,2, 3,3,3,3,3,3,3, 4,4,4,4,4,4,4, 5,5,5,5,5,5];
Col = [1,2,3,4,5, 1,2,3,4,5, 1,2,3,4,5,6,7, 1,2,3,4,5,6,7, 1,2,3,4,5,6];

% elevation angles
figure('Color', 'w'); hold on;
color = lines(5);
for r = 1:5
    plot(el(:, Row==r), 'Color', color(r,:));
end
xlabel('frames');
ylabel('angles');
title('Elevation angle by row');

% plot a vertical line every 1 second
loc = fps;
yl = ylim;
while(loc < (steps+fps))
    vline(loc, 'k:');
    text(loc, yl(1)+3, sprintf('%ds', loc/fps));
    loc = loc + fps;
    
end

% azimuth angles
% az = smoothdata(az, 'gaussian', 20);
% el = smoothdata(el, 'gaussian', 20);
figure('Color', 'w'); hold on;
color = lines(5);
for r = 1:5
    plot(az(:, Row==r), 'Color', color(r,:));
    % plot(az(:, 5));
end
xlabel('frames');
ylabel('angles');
title('Azimuthal angle by row');

% plot a vertical line every 1 second
loc = fps;
yl = ylim;
while(loc < (steps+fps))
    vline(loc, 'k:');
    text(loc, yl(1)+3, sprintf('%ds', loc/fps));
    loc = loc + fps;
    
end


% Calculate del/daz 
% (for 60Hz data)
% print out delta changes 

figure('Color', 'w'); hold on;
frames = 482:541;
% frames = 152:176;
% frames = 202:226;
fprintf("Frame: %d to %d\n", frames(1), frames(end));
for r = 1:5
    daz = az(frames(2:end), Row==r) - az(frames(1:end-1), Row==r);
    del = el(frames(2:end), Row==r) - el(frames(1:end-1), Row==r);
    daz = smoothdata(daz);
    del = smoothdata(del);
    delta = del./daz;
    
    plot(daz(:), del(:), 'o', 'Color', color(r,:));
    
    fprintf("Row %d: del/daz = %.2f +/- %.2f\n", ...
        r, mean(delta(:)), std(delta(:)));

end





