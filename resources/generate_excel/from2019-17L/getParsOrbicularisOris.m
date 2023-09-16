clear; close all;
load('follicle_pos');

node_pos = zeros(17, 3);
node_idx = cell(17,3);

% between rows
node_idx{ 8} = [22 23 29 30];
node_idx{ 9} = [21 22 28 29];
node_idx{10} = [20 21 27 28];
node_idx{11} = [19 20 26 27];
node_idx{12} = [18 19 25 26];
node_idx{13} = [17 18 24 25];
node_idx{14} = [16 17 24];
node_idx{15} = [15 21 22];
node_idx{16} = [14 15 20 21];
node_idx{17} = [13 14 19 20];
node_idx{18} = [12 13 18 19];
node_idx{19} = [11 12 17 18];
node_idx{20} = [10 11 17];
for i = 8:20
    node_pos(i, :) = mean(vec_top2D(node_idx{i}+1, :));
end

% below E row and surrounding
node_idx{ 1} = [8 29 30];
node_idx{ 2} = [9 28 29];
node_idx{ 3} = [10 27 28];
node_idx{ 4} = [11 26 27];
node_idx{ 5} = [12 25 26];
node_idx{ 6} = [13 24 25];
node_idx{ 7} = [14 16 24];

for i = 1:7
    foot = getFootOfPerpendicularToLine(node_pos(node_idx{i}(1), :),...
                                      vec_top2D(node_idx{i}(2)+1, :),...
                                      vec_top2D(node_idx{i}(3)+1, :));
    node_pos(i, :) = 2*foot - node_pos(node_idx{i}(1), :);
end

% manual fix
node_pos(2, :) = node_pos(2, :) + [0.5, 0, 0];
node_pos(4, :) = node_pos(4, :) + [-0.5, 0, 0];
node_pos(7, :) = node_pos(7, :) + [0, -1, 0];

% muscle node #1
% node_pos(1, :) = 2*node_pos(2, :) - vec_top2D(1+1, :);


% caudal outside pad
offset = [0, -5, -2];
node_pos(21, :) = node_pos(1, :) + offset;
node_pos(22, :) = node_pos(2, :) + offset;
node_pos(23, :) = node_pos(3, :) + offset;
node_pos(24, :) = node_pos(4, :) + offset;
node_pos(25, :) = node_pos(5, :) + offset;
node_pos(26, :) = node_pos(6, :) + offset;
node_pos(27, :) = node_pos(7, :) + offset;
                                                         
                             
figure; hold on;
plot3d(vec_top2D, 'ko');

plot3d(node_pos(1:20, :), 'bo');
plot3d(node_pos(21:27, :), 'ro');

axis equal

node_pos_output = node_pos([27, 1:26], :);
writematrix(node_pos_output, '../../pars_orbicularis_oris_node_pos.csv')


% start using c++ indexing
%% muscle construction (indices between muscle nodes)
constrcution_index = [
    21 1 2; 1 8 2; 8 15 2; 22 2 2; 2 9 2; 9 15 2; 23 3 2; 3 10 2; 10 16 2; 
    24 4 2; 4 11 2; 11 17 2; 25 5 2; 5 12 2; 12 18 2; 
    26 6 2; 6 13 2; 13 19 2;  0 7 2; 7 14 2; 14 20 2;
];
writematrix(constrcution_index, '../../pars_orbicularis_oris_construction_idx.csv')

%%
insertion_index = [
    % mus node, follicle1 idx, follicle2 idx (-1 if none)
    1 29 30; 2 28 29; 3 27 28; 4 26 27; 5 25 26; 6 24 25; 7 16 24;
    8 22 -1; 9 21 22; 10 20 21; 11 19 20; 12 18 19; 13 17 18; 14 10 17;
    15 15 -1; 16 14 15; 17 13 14; 18 12 13; 19 11 12; 20 10 11;
];
writematrix(insertion_index, '../../pars_orbicularis_oris_insertion_idx.csv')