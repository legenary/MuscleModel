clear; close all;
load('follicle_pos');

%%



%%
node_pos = zeros(16, 3);
node_height = zeros(16, 1);
node_idx = cell(16,3);

subcapsular_level = -1.1;
% between A and B row
node_idx{ 6} = [0 1 5 6];   node_height( 6) = subcapsular_level;
node_idx{ 7} = [1 2 6 7];   node_height( 7) = subcapsular_level;
node_idx{ 8} = [2 3 7 8];   node_height( 8) = subcapsular_level;
node_idx{ 9} = [3 4 8 9];   node_height( 9) = subcapsular_level;

for i = 6:9
    node_pos(i, :) = (1+node_height(i))/2 * mean(vec_top2D(node_idx{i}+1, :)) +...
                     (1-node_height(i))/2 * mean(vec_bot2D(node_idx{i}+1, :));
end




% above A row and below B row
node_idx{ 1} = [6 0 1];
node_idx{ 2} = [7 1 2];
node_idx{ 3} = [8 2 3];
node_idx{ 4} = [9 3 4];

node_idx{11} = [6 5 6];
node_idx{12} = [7 6 7];
node_idx{13} = [8 7 8];
node_idx{14} = [9 8 9];



for i = [1:4, 11:14]
    foot = getFootOfPerpendicularToLine(node_pos(node_idx{i}(1), :),...
               (1+node_height(node_idx{i}(1)))/2 * vec_top2D(node_idx{i}(2)+1, :) + (1-node_height(node_idx{i}(1)))/2 * vec_bot2D(node_idx{i}(2)+1, :),...
               (1+node_height(node_idx{i}(1)))/2 * vec_top2D(node_idx{i}(3)+1, :) + (1-node_height(node_idx{i}(1)))/2 * vec_bot2D(node_idx{i}(3)+1, :));
    node_pos(i, :) = 2*foot - node_pos(node_idx{i}(1), :);
end

% node 5, 10, 15
node_pos(5, :) = 2*node_pos(4, :) - node_pos(3, :);
node_pos(10, :) = 2*node_pos(9, :) - node_pos(8, :);
node_pos(15, :) = 2*node_pos(14, :) - node_pos(13, :);


% node 16
node_pos(16, :) = 2*node_pos(15, :) - node_pos(12, :);
node_pos(16, 3) = node_pos(16, 3) - 0.7;
node_pos(16, 2) = node_pos(16, 2) + 0.7;


figure; hold on;
plot3d(vec_bot2D, 'ro');
plot3d(vec_top2D, 'bo');
for i = 1:31, plot3d([vec_top2D(i,:); vec_bot2D(i,:)], 'r-'); end
plot3d(node_pos([1:16], :), 'k*');
plot3d(node_pos([5, 10, 15], :), 'ko');
axis equal


node_pos_output = node_pos([16, 1:15], :);


writematrix(node_pos_output, '../../pars_interna_profunda_node_pos.csv')


%%
constrcution_index = [
    0 5 2; 5 4 8; 4 3 8; 3 2 8; 2 1 8;
    0 10 2; 10 9 8; 9 8 8; 8 7 8; 7 6 8;
    0 15 2; 15 14 8; 14 13 8; 13 12 8; 12 11 8
];
writematrix(constrcution_index, '../../pars_interna_profunda_construction_idx.csv')

%%
insertion_index = [
    % mus node, follicle1 idx, follicle2 idx (-1 if none)
    1 0 -1; 2 1 -1; 3 2 -1; 4 3 -1; 5 4 -1;
    6 0 5; 7 1 6; 8 2 7; 9 3 8; 10 4 9;
    11 5 -1; 12 6 -1; 13 7 -1; 14 8 -1; 15 9 -1;
];

insertion_height = [
     1 subcapsular_level; 2 subcapsular_level; 3 subcapsular_level; 4 subcapsular_level; 5 subcapsular_level;
     6 subcapsular_level; 7 subcapsular_level; 8 subcapsular_level; 9 subcapsular_level; 10 subcapsular_level;
     11 subcapsular_level; 12 subcapsular_level; 13 subcapsular_level; 14 subcapsular_level; 15 subcapsular_level;

];

writematrix(insertion_index, '../../pars_interna_profunda_insertion_idx.csv')
writematrix(insertion_height, '../../pars_interna_profunda_insertion_height.csv')





