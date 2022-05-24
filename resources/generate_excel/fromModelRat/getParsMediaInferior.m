clear; close all;
load('follicle_pos');

%%
node_pos = zeros(27, 3);
node_height = zeros(27, 1);
node_idx = cell(27,3);

% between C D E row
node_idx{ 7} = [10 11 19];      node_height( 7) = 1;
node_idx{ 8} = [11 12 19 20];   node_height( 8) = 0.9;
node_idx{ 9} = [12 13 20 21];   node_height( 9) = 0.6;
node_idx{10} = [13 14 21 22];   node_height(10) = 0.3;
node_idx{11} = [14 15 22 23];   node_height(11) = 0;
node_idx{12} = [15 16 23 24];   node_height(12) = -0.3;
node_idx{13} = [16 17 24 25];   node_height(13) = -0.6;

node_idx{14} = [18 19 27];      node_height(14) = 1;
node_idx{15} = [19 20 27 28];   node_height(15) = 0.9;
node_idx{16} = [20 21 28 29];   node_height(16) = 0.6;
node_idx{17} = [21 22 29 30];   node_height(17) = 0.3;
node_idx{18} = [22 23 30 31];   node_height(18) = 0;
node_idx{19} = [23 24 31 32];   node_height(19) = -0.3;
node_idx{20} = [24 25 32 33];   node_height(20) = -0.6;



for i = 7:20
    node_pos(i, :) = (1+node_height(i))/2 * mean(vec_top2D(node_idx{i}+1, :)) +...
                     (1-node_height(i))/2 * mean(vec_bot2D(node_idx{i}+1, :));
end

% above C row and below E row
node_idx{ 1} = [8 11 12];
node_idx{ 2} = [9 12 13];
node_idx{ 3} = [10 13 14];
node_idx{ 4} = [11 14 15];
node_idx{ 5} = [12 15 16];
node_idx{ 6} = [13 16 17];

node_idx{21} = [15 27 28];
node_idx{22} = [16 28 29];
node_idx{23} = [17 29 30];
node_idx{24} = [18 30 31];
node_idx{25} = [19 31 32];
node_idx{26} = [20 32 33];



for i = [1:6, 21:26]
    foot = getFootOfPerpendicularToLine(node_pos(node_idx{i}(1), :),...
               (1+node_height(node_idx{i}(1)))/2 * vec_top2D(node_idx{i}(2)+1, :) + (1-node_height(node_idx{i}(1)))/2 * vec_bot2D(node_idx{i}(2)+1, :),...
               (1+node_height(node_idx{i}(1)))/2 * vec_top2D(node_idx{i}(3)+1, :) + (1-node_height(node_idx{i}(1)))/2 * vec_bot2D(node_idx{i}(3)+1, :));
    node_pos(i, :) = 2*foot - node_pos(node_idx{i}(1), :);
end
% some correction
node_pos(25, :) = (node_pos(24, :) + node_pos(26, :))/2;
node_pos(6, :) = 2.05*node_pos(5,:) - node_pos(4,:);

% rostral outside pad
s1 = node_pos(10, :); e1 = node_pos(11, :); 
s2 = node_pos(19, :); e2 = node_pos(20, :); 
[foot1, foot2] = getCommonFootOfPerpendicularLine(s1, e1, s2, e2);
node_pos(27, :) = mean([foot1; foot2]);
node_pos(27, 1) = node_pos(27, 1) - 2;


figure; hold on;
plot3d(vec_bot2D, 'ro');
plot3d(vec_top2D, 'bo');
for i = 1:35, plot3d([vec_top2D(i,:); vec_bot2D(i,:)], 'r-'); end
plot3d(node_pos(1:27, :), 'k*');
plot3d(node_pos([12,5,6,13], :), 'ko');
axis equal

node_pos_output = node_pos([27, 1:26], :);
writematrix(node_pos_output, '../pars_media_inferior_node_pos.csv')

%%
constrcution_index = [
    0 6; 6 5; 5 4; 4 3; 3 2; 2 1;
    0 13; 13 12; 12 11; 11 10; 10 9; 9 8; 8 7;
    0 20; 20 19; 19 18; 18 17; 17 16; 16 15; 15 14;
    0 26; 26 25; 25 24; 24 23; 23 22; 22 21
];
writematrix(constrcution_index, '../pars_media_inferior_construction_idx.csv')

%%
insertion_index = [
    % mus node, follicle1 idx, follicle2 idx (-1 if none)
    0 17 -1; 0 25 26; 0 33 34;
    1 11 -1; 2 12 -1; 3 13 -1; 4 14 -1; 5 15 -1; 6 16 -1;
    7 10 -1; 8 11 19; 9 12 20; 10 13 21; 11 14 22; 12 15 23; 13 16 24;
    14 18 -1; 15 19 27; 16 20 28; 17 21 29; 18 22 30; 19 23 31; 20 24 32;
    21 27 -1; 22 28 -1; 23 29 -1; 24 30 -1; 25 31 -1; 26 32 -1
];

insertion_height = [
    0 -0.9; 0 -0.9; 0 -0.9;
    1 0.9; 2 0.6; 3 0.3; 4 0; 5 -0.3; 6 -0.6;
    7 1; 8 0.9; 9 0.6; 10 0.3; 11 0; 12 -0.3; 13 -0.6;
    14 1; 15 0.9; 16 0.6; 17 0.3; 18 0; 19 -0.3; 20 -0.6;
    21 0.9; 22 0.6; 23 0.3; 24 0; 25 -0.3; 26 -0.6
];

writematrix(insertion_index, '../pars_media_inferior_insertion_idx.csv')
writematrix(insertion_height, '../pars_media_inferior_insertion_height.csv')








