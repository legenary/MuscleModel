clear; close all;

%% figure C: f0 change
f0 = {'f0_4', 'f0_5', 'baseline', 'f0_7', 'f0_8'};
path = GetFullPath('../../bundles_reduced_4c_different_f0');
maxDeltaTheta = zeros(length(f0), 1);
maxDeltaThetaISM = zeros(length(f0), 1);
for i = 1:length(f0)
    load([path, '/bundle_', f0{i},'.mat']);
    maxDeltaTheta(i) = range(fols_az_eyenose(:, 5));
    load([path, '/bundle_ISM_', f0{i},'.mat']);
    maxDeltaThetaISM(i) = range(fols_az_eyenose(:, 5));
end

figure('Color', 'w', 'Position', [200, 200, 300, 200]); hold on;
plot(0.4:0.1:0.8, maxDeltaTheta, 'k.-');
plot(0.4:0.1:0.8, maxDeltaThetaISM, 'k.:');
plot(0.6, maxDeltaTheta(3), 'k^');
xlim([0.4, 0.8]);
xlabel('Force scaling factor');
ylabel('max(\Delta\theta)');
ax = gca;
ax.FontSize = 10;

print('FigResult2C', '-dtiff', '-r300');


%% different activationa/deactivation
red_shades = [
    255, 42, 41; % redder
    255, 63, 62;
    255, 85, 86;
    255, 106, 105;
    255, 128, 130;
    255, 149, 150;
    255, 169, 171
    ]/255;

blue_shades = [
    15, 88, 154; % bluer
    31, 111, 184;
    30, 154, 214;
    102, 202, 217;
    149, 214, 220;
    205, 238, 250;
    ]/255;

tau_a = {'ta_005', 'baseline', 'ta_020', 'ta_030', 'ta_040', 'ta_050'};
tau_d = {'td_005', 'td_010', 'td_020', 'td_030', 'baseline', 'td_050'};
path = GetFullPath('../../bundles_reduced_4c_different_activation_deactivation');
figure('Color', 'w', 'Position', [200, 200, 300, 200]); hold on;
plot([0, 120], [0, 0], 'Color', [0.7, 0.7, 0.7]);
avg_az_tau_a = zeros(length(tau_a), 120);
for i = 1:length(tau_a)
    load([path, '/bundle_', tau_a{i},'.mat']);
    
    avg_az = zeros(3, 120);
    for c = 1:3
        startFrame = 120*c;
        endFrame = startFrame+119;
        avg_az(c, :) = fols_az_eyenose(startFrame:endFrame, 5)-fols_az_eyenose(1,5);
    end
    avg_az_tau_a(i, :) = mean(avg_az, 1);
    plot(avg_az_tau_a(i,:), 'Color', red_shades(i, :));
    if i == 2
        avg_az_baseline = avg_az_tau_a(i,:);
    end
end

avg_az_tau_d = zeros(length(tau_d), 120);
for i = 1:length(tau_d)
    load([path, '/bundle_', tau_d{i},'.mat']);
    
    avg_az = zeros(3, 120);
    for c = 1:3
        startFrame = 120*c;
        endFrame = startFrame+119;
        avg_az(c, :) = fols_az_eyenose(startFrame:endFrame, 5)-fols_az_eyenose(1,5);
    end
    avg_az_tau_d(i, :) = mean(avg_az, 1);
    plot(avg_az_tau_d(i,:), 'Color', blue_shades(i,:));
end
plot(avg_az_baseline, 'k-', 'LineWidth', 1);
xlim([0, 120]);
ylim([-25, 40]);
xlabel('Simulation times (s)')
ylabel('C2 azimuth angle \theta')
print('FigResult2D', '-dtiff', '-r300');


%% different torsional stiffness
k = {'k_35', 'k_30', 'baseline', 'k_20', 'k_15'};
zeta = {'zeta_14', 'zeta_12', 'baseline', 'zeta_8', 'zeta_6'};

path = GetFullPath('../../bundles_reduced_4c_different_torsion');
figure('Color', 'w', 'Position', [200, 200, 300, 200]); hold on;
plot([0, 120], [0, 0], 'Color', [0.7, 0.7, 0.7]);
avg_az_k = zeros(length(k), 120);
for i = 1:length(k)
    load([path, '/bundle_', k{i},'.mat']);
    
    avg_az = zeros(3, 120);
    for c = 1:3
        startFrame = 120*c;
        endFrame = startFrame+119;
        avg_az(c, :) = fols_az_eyenose(startFrame:endFrame, 5)-fols_az_eyenose(1,5);
    end
    avg_az_k(i, :) = mean(avg_az, 1);
    plot(avg_az_k(i,:), 'Color', red_shades(i, :));
    if i == 2
        avg_az_baseline = avg_az_k(i,:);
    end
end

avg_az_zeta = zeros(length(zeta), 120);
for i = 1:length(zeta)
    load([path, '/bundle_', zeta{i},'.mat']);
    
    avg_az = zeros(3, 120);
    for c = 1:3
        startFrame = 120*c;
        endFrame = startFrame+119;
        avg_az(c, :) = fols_az_eyenose(startFrame:endFrame, 5)-fols_az_eyenose(1,5);
    end
    avg_az_zeta(i, :) = mean(avg_az, 1);
    plot(avg_az_zeta(i,:), 'Color', blue_shades(i,:));
end
plot(avg_az_baseline, 'k-', 'LineWidth', 1);
xlim([0, 120]);
ylim([-25, 40]);
xlabel('Simulation times (s)')
ylabel('C2 azimuth angle \theta')
print('FigResult2E', '-dtiff', '-r300');




