clear; close all;

path = GetFullPath('../bundle_reduced_all_20c_phasing_baseline');

variable_names = {
    'ISM_03_length';
    'ISM_03_rest_length';
    'ISM_03_force';
    'ISM_03_hill_model_comps';
    'ISM_03_excitation';
    'ISM_03_activation';
    'ISM_12_length';
    'ISM_12_rest_length';
    'ISM_12_force';
    'ISM_12_hill_model_comps';
    'ISM_12_excitation';
    'ISM_12_activation';
    'N_activation'
};
for i = 0:30
    variable_names = [variable_names; sprintf('fol_%02d', i)];
end
for i = 0:33
    variable_names = [variable_names; sprintf('ISM_%02d_length', i)];
end
for i = 0:37
    variable_names = [variable_names; sprintf('N_%02d_length', i)];
    variable_names = [variable_names; sprintf('M_%02d_length', i)];
end
for i = 0:14
    variable_names = [variable_names; sprintf('PIP_%02d_length', i)];
end
for i = 0:24
    variable_names = [variable_names; sprintf('PM_%02d_length', i)];
end
for i = 0:10
    variable_names = [variable_names; sprintf('NS_%02d_length', i)];
end
for i = 0:13
    variable_names = [variable_names; sprintf('POO_%02d_length', i)];
end

for i = 1:length(variable_names)
    filepath = path + "/" + variable_names{i} + ".csv";
    if exist(filepath, 'file') == 2
        eval([variable_names{i}, ' = load(filepath);']);
    end
end
nFrame = size(fol_00, 1);

% pack them 
fols_top_bot = cell(31, 2);
for i = 0:30
    var_name = sprintf('fol_%02d', i);
    if exist(var_name, 'var') == 1
        tb = eval(var_name);
        fols_top_bot{i+1, 1} = tb(:, 1:3);
        fols_top_bot{i+1, 2} = tb(:, 4:6);
        clearvars(var_name);
    end
    if all(all(fols_top_bot{i+1, 1}==0))
        fols_top_bot{i+1, 1} = [];
        fols_top_bot{i+1, 2} = [];
    end
end
ISMs_length = nan(nFrame, 34);
for i = 0:33
    var_name = sprintf('ISM_%02d_length', i);
    if exist(var_name, 'var') == 1
        ISMs_length(:, i+1) = eval(var_name);
        clearvars(var_name);
    end
end
Ns_length = nan(nFrame, 38);
Ms_length = nan(nFrame, 38);
for i = 0:37
    var1_name = sprintf('N_%02d_length', i);
    if exist(var1_name, 'var') == 1
        Ns_length(:, i+1) = eval(var1_name);
        clearvars(var1_name);
    end
    var2_name = sprintf('M_%02d_length', i);
    if exist(var2_name, 'var') == 1
        Ms_length(:, i+1) = eval(var2_name);
        clearvars(var2_name);
    end
end
PIPs_length = nan(nFrame, 15);
for i = 0:14
    var_name = sprintf('PIP_%02d_length', i);
    if exist(var_name, 'var') == 1
        PIPs_length(:, i+1) = eval(var_name);
        clearvars(var_name);
    end
end
PMs_length = nan(nFrame, 25);
for i = 0:24
    var_name = sprintf('PM_%02d_length', i);
    if exist(var_name, 'var') == 1
        PMs_length(:, i+1) = eval(var_name);
        clearvars(var_name);
    end
end
NSs_length = nan(nFrame, 11);
for i = 0:10
    var_name = sprintf('NS_%02d_length', i);
    if exist(var_name, 'var') == 1
        NSs_length(:, i+1) = eval(var_name);
        clearvars(var_name);
    end
end
POOs_length = nan(nFrame, 14);
for i = 0:13
    var_name = sprintf('POO_%02d_length', i);
    if exist(var_name, 'var') == 1
        POOs_length(:, i+1) = eval(var_name);
        clearvars(var_name);
    end
end

if exist('ISM_03_activation', 'var') == 1
    ISM_03.activation = ISM_03_activation;
    ISM_03.excitation = ISM_03_excitation;
    ISM_03.force = ISM_03_force;
    ISM_03.hill_model_components.fPE = ISM_03_hill_model_comps(:, 1);
    ISM_03.hill_model_components.fL = ISM_03_hill_model_comps(:, 2);
    ISM_03.hill_model_components.fV = ISM_03_hill_model_comps(:, 3);
    ISM_03.rest_length = ISM_03_rest_length;
    ISM_03.length = ISMs_length(:, 4);
end

if exist('ISM_12_activation', 'var') == 1
    ISM_12.activation = ISM_12_activation;
    ISM_12.excitation = ISM_12_excitation;
    ISM_12.force = ISM_12_force;
    ISM_12.hill_model_components.fPE = ISM_12_hill_model_comps(:, 1);
    ISM_12.hill_model_components.fL = ISM_12_hill_model_comps(:, 2);
    ISM_12.hill_model_components.fV = ISM_12_hill_model_comps(:, 3);
    ISM_12.rest_length = ISM_12_rest_length;
    ISM_12.length = ISMs_length(:, 13);
end

clearvars var_name var1_name var2_name i filepath ...
    ISM_03_activation ISM_03_excitation ISM_03_force ISM_03_hill_model_comps ISM_03_rest_length...
    ISM_12_activation ISM_12_excitation ISM_12_force ISM_12_hill_model_comps ISM_12_rest_length


%% extra calculation: percent contraction
% ISM contraction are calculated for each
ISMs_percentContraction = ISMs_length ./ ISMs_length(1, :);
% extrinsic contraction are calculated together (sum)
N_percentContraction = nansum(Ns_length, 2) ./ nansum(Ns_length(1, :), 2);
M_percentContraction = nansum(Ms_length, 2) ./ nansum(Ms_length(1, :), 2);
PIP_percentContraction = nansum(PIPs_length, 2) ./ nansum(PIPs_length(1, :), 2);
PM_percentContraction = nansum(PMs_length, 2) ./ nansum(PMs_length(1, :), 2);
NS_percentContraction = nansum(NSs_length, 2) ./ nansum(NSs_length(1, :), 2);
POO_percentContraction = nansum(POOs_length, 2) ./ nansum(POOs_length(1, :), 2);

[fols_az_eyenose, fols_el_eyenose, fols_top_bot_eyenose] = readFollicleAzEl(path, 'eyenose');
for i = 1:size(fols_az_eyenose, 2)
    if all(fols_az_eyenose(:, i)==90)
        fols_az_eyenose(:, i) = nan;
    end
    if all(fols_el_eyenose(:, i)==90)
        fols_el_eyenose(:, i) = nan;
    end
    if all(all(fols_top_bot_eyenose{i, 1}==0))
        fols_top_bot_eyenose{i, 1} = [];
        fols_top_bot_eyenose{i, 2} = [];
    end
end


%% save variables if exist in workspace
save_names = {
    'fols_top_bot';
    'fols_top_bot_eyenose';
    'fols_az_eyenose';
    'fols_el_eyenose';
    'ISM_03';
    'ISM_12';
    'ISMs_length';
    'ISMs_percentContraction';
    'Ns_length';
    'N_percentContraction';
    'Ms_length';
    'M_percentContraction';
    'PIPs_length';
    'PIP_percentContraction';
    'PMs_length';
    'PM_percentContraction';
    'NSs_length';
    'NS_percentContraction';
    'POOs_length';
    'POO_percentContraction';
    'N_activation';
};
bundlepath = [path, '/bundle.mat'];
for i = 1:length(save_names)
     if exist(save_names{i}, 'var') == 1
         if exist(bundlepath, 'file') == 2
            save(bundlepath, save_names{i},'-append');
         else
            save(bundlepath, save_names{i});
         end
     end
end

