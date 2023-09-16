clear; close all;

path = GetFullPath('../bundle_full_ISM_6c_60peak');

[az, el, top_bot] = readFollicleAzEl(path, 'averagerowplane');

figure; hold on;

for i = 1:31
    plot3d(top_bot{i, 1}(1, :), 'ko');
    plot3d(top_bot{i, 2}(1, :), 'r*');
end
axis equal


load('resources/NewRatMapCleanedData.mat');
figure; hold on;
plot(Row, PhiEuler, 'ko');