clear;
close all;

steps = 60 * 4;
fol_traj = zeros(31, steps, 6);
for fol = 1:31
    filename = sprintf("../fol_pos/fol_%02d.csv", fol-1);
    fol_traj(fol, :, :) = load(filename);
    
end


%% plot 3d trajectory
figure('Color', 'w'); hold on;
for i = 1:31
    plot3d(squeeze(fol_traj(i, :, 1:3)), 'r-');
    plot3d(squeeze(fol_traj(i, :, 4:6)), 'k-');
end
axis equal

%% plot angle
figure('Color', 'w'); hold on;
for i = 1:31
    this_fol_orientation = squeeze(fol_traj(i, :, 1:3)-fol_traj(i, :, 4:6));
    this_theta = zeros(steps, 1);
    for frame = 1:steps
        temp = dot(this_fol_orientation(1,:), this_fol_orientation(frame,:))...
            /norm(this_fol_orientation(1,:))/norm(this_fol_orientation(frame,:));
        if temp>1
            temp = 1;
        end
        theta = acosd(temp);
        this_theta(frame) = theta;
    end
    plot(this_theta);
end
xlabel('frame (fps:60)')
ylabel('change of orientation');


