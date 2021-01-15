clear; close all;
load('follicle_pos');

node_pos = zeros(41, 3);
node_idx = cell(41,3);

% between rows
node_idx{ 8} = [0 1 5 6];
node_idx{ 5} = [1 2 6 7];
node_idx{ 6} = [2 3 7 8];
node_idx{ 7} = [3 4 8 9];
node_idx{12} = [5 6 10 11];
node_idx{ 9} = [6 7 11 12];
node_idx{10} = [7 8 12 13];
node_idx{11} = [8 9 13 14];
node_idx{19} = [10 11 18 19];
node_idx{13} = [11 12 19 20];
node_idx{14} = [12 13 20 21];
node_idx{15} = [13 14 21 22];
node_idx{16} = [14 15 22 23];
node_idx{17} = [15 16 23 24];
node_idx{18} = [16 17 24 25];
node_idx{26} = [18 27];
node_idx{20} = [19 20 27 28];
node_idx{21} = [20 21 28 29];
node_idx{22} = [21 22 29 30];
node_idx{23} = [22 23 30 31];
node_idx{24} = [23 24 31 32];
node_idx{25} = [24 25 32 33];
for i = 5:26
    node_pos(i, :) = mean(vec_top2D(node_idx{i}+1, :));
end

% 33 34 35
% above A row and below E row
node_idx{ 4} = [8 0 1];
node_idx{ 1} = [5 1 2];
node_idx{ 2} = [6 2 3];
node_idx{ 3} = [7 3 4];
node_idx{27} = [20 27 28];
node_idx{28} = [21 28 29];
node_idx{29} = [22 29 30];
node_idx{30} = [23 30 31];
node_idx{31} = [24 31 32];
node_idx{32} = [25 32 33];
node_idx{33} = [16 14 15];
node_idx{34} = [17 15 16];
node_idx{35} = [18 16 17];
for i = [1:4, 27:35]
    foot = getFootOfPerpendicularToLine(node_pos(node_idx{i}(1), :),...
                                      vec_top2D(node_idx{i}(2)+1, :),...
                                      vec_top2D(node_idx{i}(3)+1, :));
    node_pos(i, :) = 2*foot - node_pos(node_idx{i}(1), :);
end

% caudal outside pad
s1 = node_pos(20, :); e1 = node_pos(26, :); 
s2 = node_pos(28, :); e2 = node_pos(27, :); 
[foot1, foot2] = getCommonFootOfPerpendicularLine(s1, e1, s2, e2);
node_pos(40, :) = mean([foot1; foot2]);
len = norm(node_pos(40, :) - node_pos(26, :));

node_pos(39, :) = node_pos(19, :) + (node_pos(19, :) - node_pos(13, :))...
                                 /norm(node_pos(19, :) - node_pos(13, :))*len;
node_pos(38, :) = node_pos(12, :) + (node_pos(12, :) - node_pos(9, :))...
                                 /norm(node_pos(12, :) - node_pos(9, :))*len;
node_pos(37, :) = node_pos(8, :) + (node_pos(8, :) - node_pos(5, :))...
                                 /norm(node_pos(8, :) - node_pos(5, :))*len;
node_pos(36, :) = node_pos(4, :) + (node_pos(4, :) - node_pos(1, :))...
                                 /norm(node_pos(4, :) - node_pos(1, :))*len;
node_pos(36, 3) = mean(node_pos(37:40, 3));
                             
s1 = node_pos(4, :); e1 = node_pos(36, :); 
s2 = node_pos(27, :); e2 = node_pos(40, :); 
[foot1, foot2] = getCommonFootOfPerpendicularLine(s1, e1, s2, e2);
node_pos(41, :) = mean([foot1; foot2]);

figure; hold on;
plot3d(vec_top2D, 'ko');
plot3d(node_pos(5:26, :), 'ro');
plot3d(node_pos([1:4, 27:32], :), 'bo');

plot3d(node_pos(36:41, :), 'ro');
axis equal

node_pos_output = node_pos([41, 1:40], :);
writematrix(node_pos_output, '../maxillolabialis_node_pos.csv')

%%
constrcution_index = [
    0 36; 36 4; 4 1; 1 2; 2 3;
    0 37; 37 8; 8 5; 5 6; 6 7;
    0 38; 38 12; 12 9; 9 10; 10 11; 11 33; 33 34; 34 35;
    0 39; 39 19; 19 13; 13 14; 14 15; 15 16; 16 17; 17 18;
    0 40; 40 26; 26 20; 20 21; 21 22; 22 23; 23 24; 24 25;
    0 40; 40 27; 27 28; 28 29; 29 30; 30 31; 31 32;
];
writematrix(constrcution_index, '../maxillolabialis_construction_idx.csv')

%%
insertion_index = [
    % mus node, follicle1 idx, follicle2 idx (-1 if none)
    36 0 -1; 4 1 -1; 1 2 -1; 2 3 -1; 3 4 -1;
    37 8 12; 8 1 6; 5 2 7; 6 3 8; 7 4 9;
    38 5 10; 12 6 11; 9 7 12; 10 8 13; 11 9 14; 33 15 -1; 34 16 -1; 35 17 -1;
    39 10 18; 19 11 19; 13 12 20; 14 13 21; 15 14 22; 16 15 23; 17 16 24; 18 17 25;
    40 18 27; 26 19 27; 20 20 28; 21 21 29; 22 22 30; 23 23 31; 24 24 32; 25 25 33;
    27 28 -1; 28 29 -1; 29 30 -1; 30 31 -1; 31 32 -1; 32 33 -1
];
writematrix(insertion_index, '../maxillolabialis_insertion_idx.csv')








