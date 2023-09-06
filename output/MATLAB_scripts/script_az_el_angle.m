clear; clc

path = GetFullPath('../bundle_reduced');
[top, bot] = readFollicleTopBot(path);
[az, el] = readFollicleAzEl(path, 'eyenose');



figure; hold on;
plot(az);

% plot(el);