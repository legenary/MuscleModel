clear;
ca;

fol_traj = load('../test_output.csv');
t = size(fol_traj, 1);

figure; hold on;
subplot(311); hold on;
plot(1:t, fol_traj(:,1));
subplot(312); hold on;
plot(1:t, fol_traj(:,2));
subplot(313); hold on;
plot(1:t, fol_traj(:,3));