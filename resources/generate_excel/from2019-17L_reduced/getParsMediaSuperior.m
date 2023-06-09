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
node_pos(10, 2) = node_pos(10, 2) + 0.2;
node_pos(10, 3) = node_pos(10, 3) - 0.2;




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
node_pos(5, 2) = node_pos(5, 2) - 0.5;
node_pos(11, 3) = node_pos(11, 3) - 1;
node_pos(14, 3) = node_pos(14, 3) + 0.2;
node_pos(15, 3) = node_pos(15, 3) - 0.2;

% node 16
s1 = node_pos(4, :); e1 = node_pos(5, :); 
s2 = node_pos(14, :); e2 = node_pos(15, :); 
[foot1, foot2] = getCommonFootOfPerpendicularLine(s1, e1, s2, e2);
node_pos(16, :) = mean([foot1; foot2]);
node_pos(16, 1) = node_pos(16, 1) + 3;


figure; hold on;
plot3d(vec_bot2D, 'ro');
plot3d(vec_top2D, 'bo');
plot3d(node_pos([1:16], :), 'k*');
plot3d(node_pos([4,5,14,15,16], :), 'ro');
axis equal

node_pos_output = node_pos([16, 1:15], :);

% potential adjustment for reduced array, not sure whether apply to full
% array or not
node_pos(12, 3) = node_pos(12, 3) - 0.8; 
node_pos(13, 3) = node_pos(13, 3) - 0.5;
node_pos(14, 3) = node_pos(14, 3) - 0.5;
node_pos(16, :) = node_pos(16, :) + [0, 3, -3];
node_pos_output_reduced = node_pos([16 7 8 9 12 13 14], :);
writematrix(node_pos_output_reduced, '../../pars_media_superior_node_pos_reduced.csv')


%%
constrcution_index = [
    0 5 2; 5 4 8; 4 3 8; 3 2 8; 2 1 8;
    0 10 2; 10 9 8; 9 8 8; 8 7 8; 7 6 8;
    0 15 2; 15 14 8; 14 13 8; 13 12 8; 12 11 8
];
writematrix(constrcution_index, '../../pars_media_superior_construction_idx.csv')

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

writematrix(insertion_index, '../../pars_media_superior_insertion_idx.csv')
writematrix(insertion_height, '../../pars_media_superior_insertion_height.csv')

%% reduced
constrcution_index_reduced = [
    0 3 2; 3 2 4; 2 1 4; 0 6 2; 6 5 4; 5 4 4
];
writematrix(constrcution_index_reduced, '../../pars_media_superior_construction_idx_reduced.csv')

%% reduced
insertion_index_reduced = [
    % mus node, follicle1 idx, follicle2 idx (-1 if none)
    1 0 -1; 2 1 -1; 3 2 -1;
    4 0 3; 5 1 4; 6 2 5;
];

insertion_height_reduced = [
     1 0.5; 2 0; 3 -0.5; 
     4 0.5; 5 0; 6 -0.5;

];

writematrix(insertion_index_reduced, '../../pars_media_superior_insertion_idx_reduced.csv')
writematrix(insertion_height_reduced, '../../pars_media_superior_insertion_height_reduced.csv')





