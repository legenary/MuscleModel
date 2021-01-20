
% at the end of this script, vec_top2D means the basepoint location, which is the top end of the follicle,
% and vec_bot2D means the deeper end of the follicle.
% 
% this script generates follicle information. The location and orientation
% is based on 

clear;


load('NewRatMapModelOutput.mat');

%% load follicle length and volume
% there are 35 follicles in total, if using 40-follicle-naming-convention,
% the indices are
fol_name = {'a0','a1','a2','a3','a4','a5','a6','a7',...
    'b0','b1','b2','b3','b4','b5','b6','b7',...
    'c0','c1','c2','c3','c4','c5','c6','c7',...
    'd0','d1','d2','d3','d4','d5','d6','d7',...
    'e0','e1','e2','e3','e4','e5','e6','e7',...
    };
idx2fol40 = [1, 2, 3, 4, 5, ...
             9, 10, 11, 12, 13, ...
             17, 18, 19, 20, 21, 22, 23, 24, ... %(extra),
             25, 26, 27, 28, 29, 30, 31, 32, ...
             33, 34, 35, 36, 37, 38, 39, 40]';
%          
regPad2Resliced = load('../../../../../../_ARP/04c_Reslice/data_resliced/regPad2Reslice.mat').FolResliced;
empties = cellfun(@isempty, regPad2Resliced);
regPad2Resliced(empties) = {nan};
regPad2Length = cell2mat(regPad2Resliced(2, idx2fol40));
regPad2Volume = cell2mat(regPad2Resliced(3, idx2fol40));
regPad3Resliced = load('../../../../../../_ARP/04c_Reslice/data_resliced/regPad3Reslice.mat').FolResliced;
empties = cellfun(@isempty, regPad3Resliced);
regPad3Resliced(empties) = {nan};
regPad3Length = cell2mat(regPad3Resliced(2, idx2fol40));
regPad3Volume = cell2mat(regPad3Resliced(3, idx2fol40));
avgLength = nanmean([regPad2Length; regPad3Length]);
avgVolume = nanmean([regPad2Volume; regPad3Volume]);


% note: after this step, i = 27/34 is still nan. This is due to the indexing
% offset of "e0" follicle. It needs to be taken out.
avgLength(27) = []; avgVolume(27) = [];

% note: after this step, we're missing two follicles from the ARP data:
% "c9" and "e8". We use linear interpolation for them.
% c9:
avgLength = insertArray(avgLength, 27, interp1(1:8, avgLength(19:26), 9, 'linear', 'extrap'));
avgVolume = insertArray(avgVolume, 27, interp1(1:8, avgVolume(19:26), 9, 'linear', 'extrap'));
% e8:
avgLength = insertArray(avgLength, 35, interp1(1:7, avgLength(28:34), 8, 'linear', 'extrap'));
avgVolume = insertArray(avgVolume, 35, interp1(1:7, avgVolume(28:34), 8, 'linear', 'extrap'));



%% Fit a plane to basepoints, get orientation matrix here
P3D = modelPointsBP(1:35, :);
P2D = [zeros(size(P3D, 1), 1), modelPointsBP(1:35, 2:3)];
[~, P2Din3D, T] = procrustes(P3D, P2D, 'reflection', 0, 'scaling', 0);
% Note: P2D = T.T*(P3D'-T.c')



%% get emergence vectors in world reference frame
dir_0 = [0, -1, 0]';
l = 1.4;
dir = zeros(35, 3);

vec_top3D = P3D;
for w = 1:35
    dir(w, :) = rotz(modelThetaW(w), 'deg')*...
                rotx(modelPhiW(w), 'deg')'*...
                roty(modelZetaW(w), 'deg')* dir_0;
    vec_bot3D(w, :) = P3D(w, :) - avgLength(w)*dir(w, :);
end

% vec_bot3D = P3D - l*dir;

vec_top2D = (T.T*(vec_top3D'-T.c'))';
vec_bot2D = (T.T*(vec_bot3D'-T.c'))';

vec_top2D = vec_top2D(:, [2, 3, 1]);
vec_bot2D = vec_bot2D(:, [2, 3, 1]);
vec_top2D(:, 1) = -vec_top2D(:, 1);
vec_bot2D(:, 1) = -vec_bot2D(:, 1);

figure('Color', 'w'); hold on;
for i = 1:35
    plot3d([vec_top2D(i,:); vec_bot2D(i,:)]*1000, 'k-');
    plot3d(vec_top2D(i,:)*1000, 'ro');
%     plot3d(vec_bot2D(i,:), 'bo');
end
axis equal
xlabel('x');
ylabel('y');
zlabel('z');

%% get emergence angles in face reference frame

%  ^ y
%  |
%  |     E6 E5 E4 E3 E2 E1
%  |     D6 D5 D4 D3 D2 D1
%  |     C6 C5 C4 C3 C2 C1
%  |        B5 B4 B3 B2 B1
%  |        A5 A4 A3 A2 A1
%  +--------------------------> x

direction_vector = vec_top2D - vec_bot2D;

follicle_pos_ypr_len_vol = zeros(35, 6);
follicle_pos_ypr_len_vol(:, 1:3) = (vec_top2D + vec_bot2D)/2;

for i = 1:35
    [th, pi, r] = cart2sph(direction_vector(i, 1), direction_vector(i, 2), direction_vector(i, 3));
    follicle_pos_ypr_len_vol(i, 4:6) = [th, pi, 0];
end

follicle_pos_ypr_len_vol(:, 7) = avgLength';
follicle_pos_ypr_len_vol(:, 8) = avgVolume';

writematrix(follicle_pos_ypr_len_vol, '../../follicle_pos_ypr_len_vol.csv')
save('follicle_pos', 'vec_top2D', 'vec_bot2D');


    
    
    
    
    
    


