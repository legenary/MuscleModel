function [top, bot] = readFollicleTopBot(path) 
    
    % get number of follicle
    files = dir([path, '/fol_*.csv']);
    nFol = size(files, 1);
    
    if (nFol > 0)
        filename = files(1).folder+"/"+files(1).name;
        stepTotal = size(load(filename), 1);
    end
    
    fol_traj = zeros(nFol, stepTotal, 6);
    
    for fol = 1:nFol
        filename = files(fol).folder+"/"+files(fol).name;
        fol_traj(fol, :, :) = load(filename);
    end
            
    top_bot_original = cell(size(fol_traj, 1), 2);
    for i = 1:size(fol_traj, 1)
        top_bot_original{i, 1} = squeeze(fol_traj(i, :, 1:3))*rotz(pi/2)*roty(-pi/2);
        top_bot_original{i, 2} = squeeze(fol_traj(i, :, 4:6))*rotz(pi/2)*roty(-pi/2);
        top_bot_original{i, 1}(:,3) = -top_bot_original{i, 1}(:,3);
        top_bot_original{i, 2}(:,3) = -top_bot_original{i, 2}(:,3);
    end
    top = top_bot_original(:, 1);
    bot = top_bot_original(:, 2);

end