

% at the end of this script, vec_top2D means the basepoint location, which is the top end of the follicle,
% and vec_bot2D means the deeper end of the follicle.

% this script generates follicle information based on data of 2019-17L
% resliced mystacial pad.

clear;

follicle_pos_ypr_len_vol = zeros(31, 8);

PadResliced = load('E:\Northwestern University\Hartmann Lab - Mitra Hartmann - __YifuLuo_activeWorking\_ARP\04c_Reslice\data_resliced\2019-17LReslice1.mat').FolResliced;
% the imported indexing is (40 in total):
fol_name = {'a0','a1','a2','a3','a4','a5','a6','a7',...
    'b0','b1','b2','b3','b4','b5','b6','b7',...
    'c0','c1','c2','c3','c4','c5','c6','c7',...
    'd0','d1','d2','d3','d4','d5','d6','d7',...
    'e0','e1','e2','e3','e4','e5','e6','e7',...
    };
PadResliced(6, :) = fol_name;

% but only part of them are valid
PadResliced(:, [6, 7, 8, 14, 15, 16, 23, 24, 33]) = [];

%% get follicle top and bottom
vec_top2D = cell2mat(cellfun(@(x) x', PadResliced(4,:), 'uni', 0))';
vec_bot2D = cell2mat(cellfun(@(x) x', PadResliced(5,:), 'uni', 0))';
% sort two ends so that they're top and bottom
for i = 1:31
    if vec_top2D(i, 3) < vec_bot2D(i, 3)
        temp = vec_top2D(i, :);
        vec_top2D(i, :) = vec_bot2D(i, :);
        vec_bot2D(i, :) = temp;
    end
end
% reverse y-axis
vec_top2D(:, 2) = -vec_top2D(:, 2);
vec_bot2D(:, 2) = -vec_bot2D(:, 2);
% change unit to mm:
vec_top2D = vec_top2D/1000;
vec_bot2D = vec_bot2D/1000;

follicle_pos_ypr_len_vol(:, 1:3) = (vec_top2D + vec_bot2D) / 2;

%% get follicle length and volume
follicle_pos_ypr_len_vol(:, 7) = cell2mat(PadResliced(2, :))'; %length
follicle_pos_ypr_len_vol(:, 8) = cell2mat(PadResliced(3, :))'; %volume

%% get follicle ypr angle
direction_vector = vec_top2D - vec_bot2D;
for i = 1:31
    [th, pi, r] = cart2sph(direction_vector(i, 1), direction_vector(i, 2), direction_vector(i, 3));
    follicle_pos_ypr_len_vol(i, 4:6) = [th, pi, 0];
end


hold on;
plot3d(vec_top2D, 'ro');
plot3d(vec_bot2D, 'b.');
for i = 1:31
    plot3d([vec_top2D(i, :); vec_bot2D(i, :)], 'k-');
end
axis equal


writematrix(follicle_pos_ypr_len_vol, '../../follicle_pos_ypr_len_vol_fromARP.csv')
save('follicle_pos.mat', 'vec_top2D', 'vec_bot2D');








