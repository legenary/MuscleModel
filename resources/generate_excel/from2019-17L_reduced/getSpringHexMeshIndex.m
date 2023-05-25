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
                  
springHexMeshIdx_reduced = [
    0 1; 1 2; 3 4; 4 5; 6 7; 7 8; 0 3; 1 4; 2 5; 3 6; 4 7; 5 8;
    0 4; 1 5; 3 7; 4 8
];

writematrix(springHexMeshIdx_reduced, '../../spring_hex_mesh_idx_reduced.csv')
writematrix(springHexMeshIdx, '../../spring_hex_mesh_idx.csv')