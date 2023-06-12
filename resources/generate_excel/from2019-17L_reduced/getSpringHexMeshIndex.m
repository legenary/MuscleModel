clear;
springHexMeshIdx = [0 1;  1 2;  2 3;  3 4;   % A row
                  5 0; 0 6;  6 1;  1 7;  7 2;  2 8;  8 3;  3 9;  9 4;   % between A and B row
                  5 6; 6 7;  7 8;  8 9;   % B row
                  10 5; 5 11;  11 6;  6 12;  12 7;  7 13;  13 8;  8 14;  14 9;  9 15; % between B and C row
                  10 11;  11 12;  12 13;  13 14;  14 15;  % C row
                  16 10; 10 17; 17 11; 11 18; 18 12; 12 19; 19 13; 13 20; 20 14; 14 21; 21 15; 15 22; 15 23; % between C and D row
                  16 17; 17 18; 18 19; 19 20; 20 21; 21 22; 22 23; % D row
                  24 17; 17 25; 25 18; 18 26; 26 19; 19 27; 27 20; 20 28; 28 21; 21 29; 29 22; 22 30; 30 23; % between D and E row
                  16 24; 24 25; 25 26; 26 27; 27 28; 28 29; 29 30]; % E row
springBendingIdx = [0 2; 1 3; 2 4; 5 7; 6 8; 7 9; 10 12; 11 13; 12 14; 13 15;
    16 18; 17 19; 18 20; 19 21; 20 22; 21 23;
    24 26; 25 27; 26 28; 27 29; 28 30;  % red
    10 25; 5 18; 11 26; 0 12; 6 19; 12 27; 1 13; 7 20; 13 28;
    2 14; 8 21; 14 29; 3 15; 9 22; 15 30; % blue
    0 10; 5 16; 1 11; 6 17; 11 24; 2 12; 7 18; 12 25;
    3 13; 8 19; 13 26; 4 14; 9 20; 14 27; 15 28]; % green
                  
springHexMeshIdx_reduced = [
    0 1; 1 2; 3 4; 4 5; 6 7; 7 8; 0 3; 1 4; 2 5; 3 6; 4 7; 5 8;
    0 4; 1 5; 3 7; 4 8
];
springBendingIdx_reduced = [
    % the first 2 indices are from-index and to-index of the bending spring
    % the last 2 indices are the indices of the edges it crossed, it's used
    % in the dihedral model
    2 4 1 5; 1 3 0 4; 5 7 4 8; 4 6 3 7;
    0 5 1 4; 3 8 4 7; 1 8 4 5; 0 7 3 4
];

writematrix(springHexMeshIdx, '../../spring_hex_mesh_idx.csv')
writematrix(springBendingIdx, '../../spring_bending_idx.csv')

writematrix(springHexMeshIdx_reduced, '../../spring_hex_mesh_idx_reduced.csv')
writematrix(springBendingIdx_reduced, '../../spring_bending_idx_reduced.csv')
