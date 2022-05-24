
% at the end of this script, vec_top2D means the basepoint location, which is the top end of the follicle,
% and vec_bot2D means the deeper end of the follicle.

% this script generates follicle information based on data of averaged from
% three resliced mystacial pad - regPad2, regPad3, 2019-17L.

clear;

follicle_pos_ypr_len_vol = zeros(31, 8);

PadResliced1 = load('../../../../../../_ARP/04c_Reslice/data_resliced/2019-17LReslice.mat').FolResliced;
PadResliced2 = load('../../../../../../_ARP/04c_Reslice/data_resliced/regPad2Reslice.mat').FolResliced;
PadResliced3 = load('../../../../../../_ARP/04c_Reslice/data_resliced/regPad3Reslice.mat').FolResliced;

% the imported indexing is (31 in total):
fol_name = {'a1','a2','a3','a4','a5',...
            'b1','b2','b3','b4','b5',...
            'c1','c2','c3','c4','c5','c6',...
            'd1','d2','d3','d4','d5','d6','d7','d8',...
                 'e2','e3','e4','e5','e6','e7','e8'
    };


% but only part of them are valid
PadResliced1(:, [6, 7, 8, 14, 15, 16, 23, 24, 33]) = [];
PadResliced2(:, [6, 7, 8, 14, 15, 16, 23, 24, 33]) = [];
PadResliced3(:, [6, 7, 8, 14, 15, 16, 23, 24, 33]) = [];

%% get average follicle top and bot
vec_top2D1 = cell2mat(cellfun(@(x) x', PadResliced1(4,:), 'uni', 0))';
vec_bot2D1 = cell2mat(cellfun(@(x) x', PadResliced1(5,:), 'uni', 0))';
% sort two ends so that they're top and bottom
for i = 1:31
    if vec_top2D1(i, 3) < vec_bot2D1(i, 3)
        temp = vec_top2D1(i, :);
        vec_top2D1(i, :) = vec_bot2D1(i, :);
        vec_bot2D1(i, :) = temp;
    end
end
vec_top2D2 = cell2mat(cellfun(@(x) x', PadResliced2(4,:), 'uni', 0))';
vec_bot2D2 = cell2mat(cellfun(@(x) x', PadResliced2(5,:), 'uni', 0))';
% sort two ends so that they're top and bottom
for i = 1:31
    if vec_top2D2(i, 3) < vec_bot2D2(i, 3)
        temp = vec_top2D2(i, :);
        vec_top2D2(i, :) = vec_bot2D2(i, :);
        vec_bot2D2(i, :) = temp;
    end
end
vec_top2D3 = cell2mat(cellfun(@(x) x', PadResliced3(4,:), 'uni', 0))';
vec_bot2D3 = cell2mat(cellfun(@(x) x', PadResliced3(5,:), 'uni', 0))';
% sort two ends so that they're top and bottom
for i = 1:31
    if vec_top2D3(i, 3) < vec_bot2D3(i, 3)
        temp = vec_top2D3(i, :);
        vec_top2D3(i, :) = vec_bot2D3(i, :);
        vec_bot2D3(i, :) = temp;
    end
end

vec_top2D = mean(cat(3, vec_top2D1, vec_top2D2, vec_top2D3), 3);
vec_bot2D = mean(cat(3, vec_bot2D1, vec_bot2D2, vec_bot2D3), 3);


% reverse y-axis
vec_top2D(:, 2) = -vec_top2D(:, 2);
vec_bot2D(:, 2) = -vec_bot2D(:, 2);
% change unit to mm:
vec_top2D = vec_top2D/1000;
vec_bot2D = vec_bot2D/1000;

follicle_pos_ypr_len_vol(:, 1:3) = (vec_top2D + vec_bot2D) / 2;


%% get follicle length and volume
follicle_pos_ypr_len_vol(:, 7) = mean(cell2mat([PadResliced1(2, :); PadResliced2(2, :); PadResliced3(2, :)])); %length
follicle_pos_ypr_len_vol(:, 8) = mean(cell2mat([PadResliced1(3, :); PadResliced2(3, :); PadResliced3(3, :)])); %volume

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


writematrix(follicle_pos_ypr_len_vol, '../../follicle_pos_ypr_len_vol.csv')
save('follicle_pos.mat', 'vec_top2D', 'vec_bot2D');








