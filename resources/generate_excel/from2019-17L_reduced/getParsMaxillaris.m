clear; close all;
load('follicle_pos');

%%
subcapsular_level = -1.1;

node_pos = zeros(26, 3);
node_height = subcapsular_level * ones(26, 1);
node_idx = cell(26,3);

% between C D E row
node_idx{ 6} = [10 11 17];      node_height( 6) = subcapsular_level;
node_idx{ 7} = [11 12 17 18];   node_height( 7) = subcapsular_level;
node_idx{ 8} = [12 13 18 19];   node_height( 8) = subcapsular_level;
node_idx{ 9} = [13 14 19 20];   node_height( 9) = subcapsular_level;
node_idx{10} = [14 15 20 21];   node_height(10) = subcapsular_level;

node_idx{13} = [16 17 24];      node_height(13) = subcapsular_level;
node_idx{14} = [17 18 24 25];   node_height(14) = subcapsular_level;
node_idx{15} = [18 19 25 26];   node_height(15) = subcapsular_level;
node_idx{16} = [19 20 26 27];   node_height(16) = subcapsular_level;
node_idx{17} = [20 21 27 28];   node_height(17) = subcapsular_level;
node_idx{18} = [21 22 28 29];   node_height(18) = subcapsular_level;
node_idx{19} = [22 23 29 30];   node_height(19) = subcapsular_level;



for i = [6:10, 13:19]
    node_pos(i, :) = (1+node_height(i))/2 * mean(vec_top2D(node_idx{i}+1, :)) +...
                     (1-node_height(i))/2 * mean(vec_bot2D(node_idx{i}+1, :));
end

% above C row and below E row
node_idx{11} = [18 21 22];
node_idx{12} = [19 22 23];

node_idx{ 1} = [7 11 12];
node_idx{ 2} = [8 12 13];
node_idx{ 3} = [9 13 14];
node_idx{ 4} = [10 14 15];

node_idx{20} = [14 24 25];
node_idx{21} = [15 25 26];
node_idx{22} = [16 26 27];
node_idx{23} = [17 27 28];
node_idx{24} = [18 28 29];
node_idx{25} = [19 29 30];

for i = [1:4, 11:12, 20:25]
    foot = getFootOfPerpendicularToLine(node_pos(node_idx{i}(1), :),...
               (1+node_height(node_idx{i}(1)))/2 * vec_top2D(node_idx{i}(2)+1, :) + (1-node_height(node_idx{i}(1)))/2 * vec_bot2D(node_idx{i}(2)+1, :),...
               (1+node_height(node_idx{i}(1)))/2 * vec_top2D(node_idx{i}(3)+1, :) + (1-node_height(node_idx{i}(1)))/2 * vec_bot2D(node_idx{i}(3)+1, :));
    node_pos(i, :) = 2*foot - node_pos(node_idx{i}(1), :);
end
node_pos(5, :) = 2*node_pos(4 ,:) - node_pos(3, :);
node_pos(5, 1) = node_pos(5, 1) - 0.6;
node_pos(11, 1) = node_pos(11, 1) - 0.6;
node_pos(12, 1) = node_pos(12, 1) - 0.6;
node_pos(11, 3) = node_pos(11, 3) + 0.1;
node_pos(12, 3) = node_pos(12, 3) + 0.1;
node_pos(20, 1) = node_pos(20, 1) + 0.5;
node_pos(24, 3) = node_pos(24, 3) - 0.5;



% rostral outside pad
s1 = node_pos(11, :); e1 = node_pos(12, :); 
s2 = node_pos(18, :); e2 = node_pos(19, :); 
[foot1, foot2] = getCommonFootOfPerpendicularLine(s1, e1, s2, e2);
node_pos(27, :) = mean([foot1; foot2]);
node_pos(27, 1) = node_pos(27, 1) - 3;

for i = 1:27, node_pos(i, 1) = node_pos(i, 1) - 0.5; end


node_pos_output = node_pos([27, 1:26], :);
% potential adjustment for reduced array, not sure whether apply to full
% array or not
node_pos(1, :) = node_pos(1, :) + [0, 0, 0.1];
node_pos(7, :) = node_pos(7, :) + [0.2, 0, 0.2];
node_pos(7, :) = node_pos(7, :) + [0, 0, 0.1];
node_pos(16, :) = node_pos(16, :) + [0.2, -0.2, 0.2];
node_pos(15, :) = node_pos(15, :) + [0.3, -0.3, 0.3];
node_pos(14, :) = node_pos(14, :) + [0.7, -0.5, 0.4];
node_pos_output_reduced = node_pos([27 1 2 3 7 8 9 14 15 16], :);
writematrix(node_pos_output_reduced, '../../pars_maxillaris_node_pos_reduced.csv')
writematrix(node_pos_output, '../../pars_maxillaris_node_pos.csv')

figure; hold on;
plot3d(vec_bot2D, 'ro');
plot3d(vec_top2D, 'bo');
for i = 1:31, plot3d([vec_top2D(i,:); vec_bot2D(i,:)], 'r-'); end
plot3d(node_pos(1:27, :), 'k*');
plot3d(node_pos([5], :), 'ro');
axis equal

%%
constrcution_index = [
    0 5; 5 4; 4 3; 3 2; 2 1;
    0 12; 12 11; 11 10; 10 9; 9 8; 8 7; 7 6;
    0 19; 19 18; 18 17; 17 16; 16 15; 15 14; 14 13;
    0 25; 25 24; 24 23; 23 22; 22 21; 21 20
];
writematrix(constrcution_index, '../../pars_maxillaris_construction_idx.csv')

insertion_index = [
    % mus node, follicle1 idx, follicle2 idx (-1 if none)
    0 23 30;
    1 11 -1; 2 12 -1; 3 13 -1; 4 14 -1; 5 15 -1;
    6 10 -1; 7 11 17; 8 12 18; 9 13 19; 10 14 20; 11 21 -1; 12 22 15;
    13 16 -1; 14 17 24; 15 18 25; 16 19 26; 17 20 27; 18 21 28; 19 22 29;
    20 24 -1; 21 25 -1; 22 26 -1; 23 27 -1; 24 28 -1; 25 29 -1;
];

insertion_height = [
    0 subcapsular_level;
    1 subcapsular_level; 2 subcapsular_level; 3 subcapsular_level; 4 subcapsular_level; 5 subcapsular_level; 
    6 subcapsular_level; 7 subcapsular_level; 8 subcapsular_level; 9 subcapsular_level; 10 subcapsular_level; 11 subcapsular_level; 12 subcapsular_level; 
    13 subcapsular_level; 14 subcapsular_level; 15 subcapsular_level; 16 subcapsular_level; 17 subcapsular_level; 18 subcapsular_level; 19 subcapsular_level; 
    20 subcapsular_level; 21 subcapsular_level; 22 subcapsular_level; 23 subcapsular_level; 24 subcapsular_level; 25 subcapsular_level;
];

writematrix(insertion_index, '../../pars_maxillaris_insertion_idx.csv')
writematrix(insertion_height, '../../pars_maxillaris_insertion_height.csv')

%% reduced array
constrcution_index_reduced = [
    0 3 2; 3 2 4; 2 1 4; 0 6 2; 6 5 4; 5 4 4
    0 9 2; 9 8 4; 8 7 4
];
writematrix(constrcution_index_reduced, '../../pars_maxillaris_construction_idx_reduced.csv')

insertion_index_reduced = [
    % mus node, follicle1 idx, follicle2 idx (-1 if none)
    3 2 5; 2 1 4; 1 0 3; 6 5 8; 5 4 7; 4 3 6;
    9 8 -1; 8 7 -1; 7 6 -1
];

insertion_height_reduced = [
    1 subcapsular_level; 2 subcapsular_level; 3 subcapsular_level; 
    4 subcapsular_level; 5 subcapsular_level; 6 subcapsular_level; 
    7 subcapsular_level; 8 subcapsular_level; 9 subcapsular_level
];

writematrix(insertion_index_reduced, '../../pars_maxillaris_insertion_idx_reduced.csv')
writematrix(insertion_height_reduced, '../../pars_maxillaris_insertion_height_reduced.csv')








