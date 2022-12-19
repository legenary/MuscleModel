clear; close all;
warning off;

%% load data
fps = 120;
is = 100;
path = sprintf('../OLD_varying_fiber_query_rate/bundle_%dHz_%d/', fps, is);
nFol = 31;
fol_pos = nan(nFol, 6, 1000);
for i = 1:nFol
    idx = i - 1;
    
    this_fol = load(sprintf('%sfol_%02d.csv', path, idx));
    fol_pos(i, :, 1:length(this_fol)) = reshape(this_fol', 1, 6, length(this_fol));

end
totalFrame = find(isnan(squeeze(fol_pos(1, 1, :))), 1, 'first') - 1;

load('resources\2019-17LReslice1.mat');
folMapping = [1:5, 9:13, 17:22, 25:32, 34:40];

muscle_pair_idx = [
    0	1; 1	2; 2	3; 3	4; 0	6; 5	6; 6	7; 7	8
    8	9; 5	11; 10	11; 11	12; 12	13; 13	14; 14	15; 10	17
    16	17; 17	18; 18	19; 19	20; 20	21; 21	22; 22	23; 16	24
    24	25; 25	26; 26	27; 27	28; 28	29; 29	30
] + 1;

%% draw frame
figure('Position', [200, 200, 800, 600], 'Color', 'w');
F(totalFrame) = getframe(gcf);  
F(totalFrame).cdata = [];
ax = gca;
ax.XLim = [-6, 9];
ax.YLim = [-5, 6];
ax.ZLim = [-7.5, 3];
ax.DataAspectRatio = [1, 1, 1];
ax.View = [15, 22];
% ax.View = [0, 90];
vpm = viewmtx(ax.View(1), ax.View(2)); % camera view projection matrix
whiskerLen = 3;
for f = 1:totalFrame
    delete(findobj('Type', 'Line'));
    delete(findobj('Type', 'Patch'));
    hold on;
    % plot follicles
    cellLine = cell(nFol, 1);
    for i = 1:nFol
        xLine = [fol_pos(i, 1, f), fol_pos(i, 4, f)];
        yLine = [fol_pos(i, 2, f), fol_pos(i, 5, f)];
        zLine = [fol_pos(i, 3, f), fol_pos(i, 6, f)];
        cellLine{i} = [xLine; yLine; zLine];
        
        % realign and draw 3D follicle
        reslicedTop = FolResliced{4, folMapping(i)}/1000;
        reslicedBot = FolResliced{5, folMapping(i)}/1000;
        
        upVec = [0, 0, 0.1];
        S = [reslicedTop; reslicedBot];
        D = [xLine', yLine', zLine'];
        [~, ~, trans] = procrustes(...
            [D; (D(1,:)+D(2,:))/2 + upVec],...  % destination
            [S; (S(1,:)+S(2,:))/2 + upVec],...  % source
            'reflection', false, 'scaling', false);
        % Z = bST + c;
        transformedFol = cellfun(@(x) trans.b * x./1000 * trans.T + trans.c(1,:),...
            FolResliced{1, folMapping(i)}, 'uni', 0);
        transformedFolXYZ = cell2mat(transformedFol);
        transformedFolXYZW = [transformedFolXYZ, ones(length(transformedFolXYZ), 1)];
%         shp = alphaShape(transformedFolXYZ(:,1),...
%             transformedFolXYZ(:,2), ...
%             transformedFolXYZ(:,3));
%         plot(shp, 'EdgeColor', 'none', 'FaceAlpha', 0.7);

        vLine = -[xLine(2)-xLine(1), yLine(2)-yLine(1), zLine(2)-zLine(1)];
        wh = [0, 0, 0; vLine] /norm(vLine) * whiskerLen;
        plot3(xLine, yLine, zLine, 'r:');              
        plot3(xLine(1) + wh(:,1), ...        
            yLine(1) + wh(:,2), ...
            zLine(1) + wh(:,3),...                                 
            'r-', 'LineWidth', 2); 
        plot3dScreenOutline(transformedFolXYZ(:,1), transformedFolXYZ(:,2),...
            transformedFolXYZ(:,3), 'k', 'FaceAlpha', 0.1, 'EdgeColor', 'k');
    end
    
    % draw intrinsic muscle
    for i = 1:length(muscle_pair_idx)
        folFrom = cellLine{muscle_pair_idx(i, 1)};
        ptFrom = folFrom(:, 1);
        folTo = cellLine{muscle_pair_idx(i, 2)};
        ptTo = folTo(:,2)*0.8 + folTo(:,1)*0.2;

        plot3([ptFrom(1), ptTo(1)],...
            [ptFrom(2), ptTo(2)],...
            [ptFrom(3), ptTo(3)], 'b-');
    end

    hold off
%     drawnow
    
    F(f) = getframe(gcf);
    
end

%% output video
writerObj = VideoWriter('myVideo');
writerObj.FrameRate = fps/1.5;
open(writerObj);
for i=1:length(F)
    writeVideo(writerObj, F(i));
end
close(writerObj);





