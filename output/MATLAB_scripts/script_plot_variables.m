clear; close all;

path = GetFullPath('../bundle_reduced');

variable_names = {
    'ISM_0';
    'Fol00_top'
};


for i = 1:length(variable_names)
    filepath = path + "/" + variable_names{i} + ".csv";
    if exist(filepath, 'file') == 2
        eval([variable_names{i}, ' = load(filepath);']);
        figure; hold on;
        eval(['plot(', variable_names{i}, ');']);
        title(variable_names{i}, 'Interpreter', 'none');
    end
end