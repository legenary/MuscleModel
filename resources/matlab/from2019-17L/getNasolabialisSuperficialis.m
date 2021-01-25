clear; close all;
load('follicle_pos');

node_pos = zeros(17, 3);
node_idx = cell(17,3);

% between rows
node_idx{ 7} = [0 5 6];
node_idx{ 8} = [1 2 6 7];
node_idx{ 9} = [2 3 7 8];
node_idx{10} = [3 4 8 9];
for i = 7:10
    node_pos(i, :) = mean(vec_top2D(node_idx{i}+1, :));
end

% above A row and surrounding
node_idx{ 2} = [7 0 1];
node_idx{ 3} = [8 1 2];
node_idx{ 4} = [9 2 3];
node_idx{ 5} = [10 3 4];
node_idx{11} = [10 4 9];
node_idx{ 6} = [11 3 4];

for i = [2:5, 11, 6]
    foot = getFootOfPerpendicularToLine(node_pos(node_idx{i}(1), :),...
                                      vec_top2D(node_idx{i}(2)+1, :),...
                                      vec_top2D(node_idx{i}(3)+1, :));
    node_pos(i, :) = 2*foot - node_pos(node_idx{i}(1), :);
end

node_pos(3, :) = node_pos(8, :) + node_pos(4, :) - node_pos(9, :);
node_pos(2, :) = node_pos(7, :) + node_pos(4, :) - node_pos(9, :);

% muscle node #1
node_pos(1, :) = 2*node_pos(2, :) - vec_top2D(1+1, :);


% caudal outside pad
offset = [0, 5, -2];
node_pos(12, :) = node_pos(2, :) + offset;
node_pos(13, :) = node_pos(3, :) + offset;
node_pos(14, :) = node_pos(4, :) + offset;
node_pos(15, :) = node_pos(5, :) + offset;
node_pos(16, :) = node_pos(6, :) + offset;
node_pos(17, :) = node_pos(1, :) + offset;
                             
                             
figure; hold on;
plot3d(vec_top2D, 'ko');

plot3d(node_pos(7:10, :), 'bo');
plot3d(node_pos(1:17, :), 'bo');
plot3d(node_pos([1], :), 'ro');

axis equal

node_pos_output = node_pos([17, 1:16], :);
writematrix(node_pos_output, '../../nasolabialis_superficialis_node_pos.csv')


% start using c++ indexing
%% muscle construction (indices between muscle nodes)
constrcution_index = [
    0 1; 12 2; 2 7; 13 3; 3 8; 14 4; 4 9; 15 5; 5 10; 16 6; 6 11;
];
writematrix(constrcution_index, '../../nasolabialis_superficialis_construction_idx.csv')

%%
insertion_index = [
    % mus node, follicle1 idx, follicle2 idx (-1 if none)
    1 0 5; 2 0 1; 3 1 2; 4 2 3; 5 3 4; 6 4 -1; 7 5 6; 8 6 7; 9 7 8; 10 8 9; 11 9 -1;
];
writematrix(insertion_index, '../../nasolabialis_superficialis_insertion_idx.csv')