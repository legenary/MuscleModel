clear; close all;
load('follicle_pos');

%%



%%
node_pos = zeros(16, 3);
node_height = zeros(16, 1);
node_idx = cell(16,3);

% between A and B row
node_idx{ 6} = [0 1 5 6];   node_height( 6) = 1;
node_idx{ 7} = [1 2 6 7];   node_height( 7) = 0.5;
node_idx{ 8} = [2 3 7 8];   node_height( 8) = 0;
node_idx{ 9} = [3 4 8 9];   node_height( 9) = -0.5;

for i = 6:9
    node_pos(i, :) = (1+node_height(i))/2 * mean(vec_top2D(node_idx{i}+1, :)) +...
                     (1-node_height(i))/2 * mean(vec_bot2D(node_idx{i}+1, :));
end

% node 10
node_idx{10} = [9 4 9];     node_height(10) = -0.5;
foot = getFootOfPerpendicularToLine(node_pos(node_idx{10}(1), :),...
           (1+node_height(9))/2 * vec_top2D(node_idx{10}(2)+1, :) + (1-node_height(9))/2 * vec_bot2D(node_idx{10}(2)+1, :),...
           (1+node_height(9))/2 * vec_top2D(node_idx{10}(3)+1, :) + (1-node_height(9))/2 * vec_bot2D(node_idx{10}(3)+1, :));
node_pos(10, :) = 2*foot - node_pos(node_idx{10}(1), :);



% above A row and below B row
node_idx{ 1} = [6 0 1];
node_idx{ 2} = [7 1 2];
node_idx{ 3} = [8 2 3];
node_idx{ 4} = [9 3 4];
node_idx{ 5} = [10 3 4];
node_idx{11} = [6 5 6];
node_idx{12} = [7 6 7];
node_idx{13} = [8 7 8];
node_idx{14} = [9 8 9];
node_idx{15} = [10 8 9];


for i = [1:5, 11:15]
    foot = getFootOfPerpendicularToLine(node_pos(node_idx{i}(1), :),...
               (1+node_height(node_idx{i}(1)))/2 * vec_top2D(node_idx{i}(2)+1, :) + (1-node_height(node_idx{i}(1)))/2 * vec_bot2D(node_idx{i}(2)+1, :),...
               (1+node_height(node_idx{i}(1)))/2 * vec_top2D(node_idx{i}(3)+1, :) + (1-node_height(node_idx{i}(1)))/2 * vec_bot2D(node_idx{i}(3)+1, :));
    node_pos(i, :) = 2*foot - node_pos(node_idx{i}(1), :);
end


% node 16
node_pos(16, :) = node_pos(15, :) + 3*(node_pos(15, :) - node_pos(14, :));
                             
node_pos(16, 3) = -0.5;

figure; hold on;
plot3d(vec_bot2D, 'ro');
plot3d(vec_top2D, 'bo');
plot3d(node_pos([1:16], :), 'k*');
axis equal

node_pos_output = node_pos([16, 1:15], :);
writematrix(node_pos_output, '../pars_media_superior_node_pos.csv')


%%
constrcution_index = [
    0 5; 5 4; 4 3; 3 2; 2 1;
    0 10; 10 9; 9 8; 8 7; 7 6;
    0 15; 15 14; 14 13; 13 12; 12 11
];
writematrix(constrcution_index, '../pars_media_superior_construction_idx.csv')

%%
insertion_index = [
    % mus node, follicle1 idx, follicle2 idx (-1 if none)
    1 0 -1; 2 1 -1; 3 2 -1; 4 3 -1; 5 4 -1;
    6 0 5; 7 1 6; 8 2 7; 9 3 8; 10 4 9;
    11 5 -1; 12 6 -1; 13 7 -1; 14 8 -1; 15 9 -1;
];

insertion_height = [
     1 1; 2 0.5; 3 0; 4 -0.5; 5 -0.5;
     6 1; 7 0.5; 8 0; 9 -0.5; 10 -0.5;
     11 1; 12 0.5; 13 0; 14 -0.5; 15 -0.5;

];

writematrix(insertion_index, '../pars_media_superior_insertion_idx.csv')
writematrix(insertion_height, '../pars_media_superior_insertion_height.csv')





