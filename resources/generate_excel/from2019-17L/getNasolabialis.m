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
% writematrix(node_pos_output, '../../nasolabialis_node_pos.csv')


%%
constrcution_index = [
    0 34; 34 1; 1 2; 2 3; 34 4; 4 5; 5 6; 6 7;
    0 35; 35 8; 8 9; 9 10; 10 11; 11 33; 
    0 36; 36 12; 12 13; 13 14; 14 15; 15 16; 16 17; 17 18;
    0 37; 37 19; 19 20; 20 21; 21 22; 22 23; 23 24; 24 25;
    0 38; 38 26; 26 27; 27 28; 28 29; 29 30; 30 31; 31 32;
];
% writematrix(constrcution_index, '../../nasolabialis_construction_idx.csv')

%%
insertion_index = [
    % mus node, follicle1 idx, follicle2 idx (-1 if none)
    34 0 1; 1 2 -1; 2 3 -1; 3 4 -1;
    4 1 6; 5 2 7; 6 3 8; 7 4 9;
    35 0 5; 8 6 11; 9 7 12; 10 8 13; 11 9 14; 33 15 -1;
    36 5 10; 12 11 17; 13 12 18; 14 13 19; 15 14 20; 16 15 21; 17 22 -1; 18 23 -1;
    37 10 16; 19 17 24; 20 18 25; 21 19 26; 22 20 27; 23 21 28; 24 22 29; 25 23 30;
    38 16 -1; 26 24 -1; 27 25 -1; 28 26 -1; 29 27 -1; 30 28 -1; 31 29 -1; 32 30 -1;
];
% writematrix(insertion_index, '../../nasolabialis_insertion_idx.csv')