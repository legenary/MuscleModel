clear;
close all;

steps = 60 * 4;
fol_traj = zeros(31, steps, 6);
for fol = 1:31
    filename = sprintf("../fol_pos/fol_%02d.csv", fol-1);
    fol_traj(fol, :, :) = load(filename);
    
end

%% plot 3d trajectory
R = [0.9092,0.3570,0.2142;
    -0.3598,0.9326,-0.0270;
    -0.2094,-0.0525,0.9764];

figure('Color', 'w'); hold on;
top_bot = cell(31, 2);

% adjust to world reference frame
for i = 1:31
    top_bot{i, 1} = squeeze(fol_traj(i, :, 1:3))*rotz(pi/2)*roty(-pi/2);
    top_bot{i, 2} = squeeze(fol_traj(i, :, 4:6))*rotz(pi/2)*roty(-pi/2);
    top_bot{i, 1}(:,3) = -top_bot{i, 1}(:,3);
    top_bot{i, 2}(:,3) = -top_bot{i, 2}(:,3);
    plot3d(top_bot{i, 1}, 'ro');

    plot3d(top_bot{i, 1}*R, 'bo');
    plot3d(top_bot{i, 2}*R, 'b-');
end

axis equal
xlabel('x')
ylabel('y')

%% Generate az and el angle
az = zeros(steps, 31);
el = zeros(steps, 31);

for i = 1:31
    dir = top_bot{i, 1} - top_bot{i, 2};
    [th, phi, r] = cart2sph(dir(:,1), dir(:,2), dir(:,3));
    
    az(:, i) = rad2deg(th);
    el(:, i) = rad2deg(phi);
    
end

Row = [1,1,1,1,1, 2,2,2,2,2, 3,3,3,3,3,3,3, 4,4,4,4,4,4,4, 5,5,5,5,5,5];
Col = [1,2,3,4,5, 1,2,3,4,5, 1,2,3,4,5,6,7, 1,2,3,4,5,6,7, 1,2,3,4,5,6];

figure('Color', 'w'); hold on;
color = lines(5);
for r = 1:5
    plot(el(:, Row==r), 'Color', color(r,:));
end
xlabel('frames');
ylabel('angles');
title('Elevation angle by row');

figure('Color', 'w'); hold on;
color = lines(5);
for r = 1:5
    plot(az(:, Row==r), 'Color', color(r,:));
end
xlabel('frames');
ylabel('angles');
title('Azimuthal angle by row');

%% Use frame 103-127 to calculate del/daz
% frames = 102:126;
frames = 152:176;
figure('Color', 'w'); hold on;
for r = 1:5
    daz = az(frames(2:end), Row==r) - az(frames(1:end-1), Row==r);
    del = el(frames(2:end), Row==r) - el(frames(1:end-1), Row==r);
    delta = del./daz;
    
    plot(daz(:), del(:), 'o', 'Color', color(r,:));
    
    fprintf("Row %d: del/daz = %.2f +/- %.2f\n", ...
        r, mean(delta(:)), std(delta(:)));

end







