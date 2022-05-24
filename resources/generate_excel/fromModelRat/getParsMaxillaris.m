clear; close all;
load('follicle_pos');

%%
subcapsular_level = -1.4;

node_pos = zeros(27, 3);
node_height = subcapsular_level * ones(27, 1);
node_idx = cell(27,3);

subcapsular_level = -1.4;% between C D E row
node_idx{ 7} = [10 11 19];      
node_idx{ 8} = [11 12 19 20];   
node_idx{ 9} = [12 13 20 21];   
node_idx{10} = [13 14 21 22];   
node_idx{11} = [14 15 22 23];   
node_idx{12} = [15 16 23 24];   
node_idx{13} = [16 17 24 25];   

node_idx{14} = [18 19 27];     
node_idx{15} = [19 20 27 28];  
node_idx{16} = [20 21 28 29]; 
node_idx{17} = [21 22 29 30];  
node_idx{18} = [22 23 30 31]; 
node_idx{19} = [23 24 31 32]; 
node_idx{20} = [24 25 32 33];  



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


% rostral outside pad
s1 = node_pos(12, :); e1 = node_pos(13, :); 
s2 = node_pos(25, :); e2 = node_pos(26, :); 
[foot1, foot2] = getCommonFootOfPerpendicularLine(s1, e1, s2, e2);
node_pos(27, :) = mean([foot1; foot2]);


figure; hold on;
plot3d(vec_bot2D, 'ro');
plot3d(vec_top2D, 'bo');
plot3d(node_pos(1:27, :), 'k*');
axis equal

node_pos_output = node_pos([27, 1:26], :);
writematrix(node_pos_output, '../pars_maxillaris_node_pos.csv')

%%
constrcution_index = [
    0 6; 6 5; 5 4; 4 3; 3 2; 2 1;
    0 13; 13 12; 12 11; 11 10; 10 9; 9 8; 8 7;
    0 20; 20 19; 19 18; 18 17; 17 16; 16 15; 15 14;
    0 26; 26 25; 25 24; 24 23; 23 22; 22 21
];
writematrix(constrcution_index, '../pars_maxillaris_construction_idx.csv')

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
    0 subcapsular_level; 0 subcapsular_level; 0 subcapsular_level;
    1 subcapsular_level; 2 subcapsular_level; 3 subcapsular_level; 4 subcapsular_level; 5 subcapsular_level; 6 subcapsular_level;
    7 subcapsular_level; 8 subcapsular_level; 9 subcapsular_level; 10 subcapsular_level; 11 subcapsular_level; 12 subcapsular_level; 13 subcapsular_level;
    14 subcapsular_level; 15 subcapsular_level; 16 subcapsular_level; 17 subcapsular_level; 18 subcapsular_level; 19 subcapsular_level; 20 subcapsular_level;
    21 subcapsular_level; 22 subcapsular_level; 23 subcapsular_level; 24 subcapsular_level; 25 subcapsular_level; 26 subcapsular_level
];

writematrix(insertion_index, '../pars_maxillaris_insertion_idx.csv')
writematrix(insertion_height, '../pars_maxillaris_insertion_height.csv')








