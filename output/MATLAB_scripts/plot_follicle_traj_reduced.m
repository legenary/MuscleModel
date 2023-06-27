clear; close all;

T = 5;
fps = 120;
clr = lines(9);

path = sprintf('../bundle_reduced');
stepTotal = size(load([path, '/fol_00.csv']), 1);

fol_traj = zeros(9, stepTotal, 6);
for fol = 1:9
    filename = sprintf('%s/fol_%02d.csv', path, fol-1);
    fol_traj(fol, :, :) = load(filename);

end

[az, el, top_bot_avg_row, top_bot_eyenose] = getAzElEyenose(fol_traj);

figure; hold on;
for frame = 1:20:599
    for fol = 1:9
        top = top_bot_eyenose{fol, 1}(frame, :);
        bot = top_bot_eyenose{fol, 2}(frame, :);
        plot3d([top; bot], 'k', 'Color', clr(fol,:));
        plot3d(top, 'ro');
    end
    
end
axis equal
figure; hold on;
plot(az);