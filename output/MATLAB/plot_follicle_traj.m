clear;
ca;

fol_traj = zeros(31, 120, 6);
for fol = 1:31
    filename = sprintf("../fol_pos/fol_%02d.csv", fol-1);
    fol_traj(fol, :, :) = load(filename);
    
end

figure; hold on;
for i = 1:31
    plot3d(squeeze(fol_traj(i, :, 1:3)), 'r-');
    plot3d(squeeze(fol_traj(i, :, 4:6)), 'k-');
end
axis equal