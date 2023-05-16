clear; close all;
load('follicle_pos');

node_pos = zeros(39, 3);
node_idx = cell(39,3);


node_idx{ 4} = [0 1];
node_idx{ 5} = [1 2 6 7];
node_idx{ 6} = [2 3 7 8];
node_idx{ 7} = [3 4 8 9];
node_idx{ 8} = [5 6];
node_idx{ 9} = [6 7 11 12];
node_idx{10} = [7 8 12 13];
node_idx{11} = [8 9 13 14];
node_idx{12} = [10 11];
node_idx{13} = [11 12 17 18];
node_idx{14} = [12 13 18 19];
node_idx{15} = [13 14 19 20];
node_idx{16} = [14 15 20 21];

node_idx{19} = [16 17];
node_idx{20} = [17 18 24 25];
node_idx{21} = [18 19 25 26];
node_idx{22} = [19 20 26 27];
node_idx{23} = [20 21 27 28];
node_idx{24} = [21 22 28 29];
node_idx{25} = [22 23 29 30];

for i = [4:16, 19:25]
    node_pos(i, :) = mean(vec_top2D(node_idx{i}+1, :));
end





% 33 34 35
% above A row and below E row
node_idx{ 1} = [5 1 2];
node_idx{ 2} = [6 2 3];
node_idx{ 3} = [7 3 4];
node_idx{26} = [19 16 24];
node_idx{27} = [20 24 25];
node_idx{28} = [21 25 26];
node_idx{29} = [22 26 27];
node_idx{30} = [23 27 28];
node_idx{31} = [24 28 29];
node_idx{32} = [25 29 30];

node_idx{33} = [16 14 15];
node_idx{17} = [24 21 22];
node_idx{18} = [25 22 23];
for i = [1:3, 26:33, 17:18]
    foot = getFootOfPerpendicularToLine(node_pos(node_idx{i}(1), :),...
                                      vec_top2D(node_idx{i}(2)+1, :),...
                                      vec_top2D(node_idx{i}(3)+1, :));
    node_pos(i, :) = 2*foot - node_pos(node_idx{i}(1), :);
end


% caudal outside pad
s1 = node_pos(2, :); e1 = node_pos(1, :); 
s2 = node_pos(6, :); e2 = node_pos(4, :); 
[foot1, foot2] = getCommonFootOfPerpendicularLine(s1, e1, s2, e2);
node_pos(34, :) = mean([foot1; foot2]);
len = norm(node_pos(34, :) - node_pos(4, :));

node_pos(35, :) = node_pos(8, :) + (node_pos(8, :) - node_pos(9, :))...
                                 /norm(node_pos(8, :) - node_pos(9, :))*len;
node_pos(36, :) = node_pos(12, :) + (node_pos(12, :) - node_pos(13, :))...
                                 /norm(node_pos(12, :) - node_pos(13, :))*len;
node_pos(37, :) = node_pos(19, :) + (node_pos(19, :) - node_pos(20, :))...
                                 /norm(node_pos(19, :) - node_pos(20, :))*len;
node_pos(38, :) = node_pos(26, :) + (node_pos(26, :) - node_pos(27, :))...
                                 /norm(node_pos(26, :) - node_pos(27, :))*len;

s1 = node_pos(1, :); e1 = node_pos(34, :); 
s2 = node_pos(26, :); e2 = node_pos(38, :); 
[foot1, foot2] = getCommonFootOfPerpendicularLine(s1, e1, s2, e2);
node_pos(39, :) = mean([foot1; foot2]);
                             
                             
figure; hold on;
plot3d(vec_top2D, 'ko');
plot3d(node_pos(4:25, :), 'ro');
plot3d(node_pos([1:3, 26:35, 17:18, 33], :), 'bo');

plot3d(node_pos(34:39, :), 'ro');
axis equal

node_pos_output = node_pos([39, 1:38], :);
node_pos_output_reduced = node_pos([39 4 5 8 9 12 13 19 20 35 36 37], :);
writematrix(node_pos_output_reduced, '../../nasolabialis_node_pos_reduced.csv')


%%
constrcution_index_reduced = [
    0 9 2; 9 1 2; 1 2 2; 9 3 2; 3 4 2; 
    0 10 2; 10 5 4; 5 6 4;
    0 11 2; 11 7 4; 7 8 4
];
writematrix(constrcution_index_reduced, '../../nasolabialis_construction_idx_reduced.csv')

%%
insertion_index_reduced = [
    % mus node, follicle1 idx, follicle2 idx (-1 if none)
    2 2 -1; 1 1 -1; 9 0 -1; 4 2 5; 3 1 4; 10 0 3; 5 4 6; 6 5 7; 11 3 -1; 7 6 -1; 8 7 -1
];
writematrix(insertion_index_reduced, '../../nasolabialis_insertion_idx_reduced.csv')