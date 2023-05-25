clear;
intrinsicSlingMuscleIdx = [0, 1; 1, 2; 2, 3; 3, 4;
                           0, 6; 5, 6; 6, 7; 7, 8; 8, 9;
                           5, 11; 10, 11; 11, 12; 12, 13; 13, 14; 14, 15; 
                           10, 17; 16, 17; 17, 18; 18, 19; 19, 20; 20, 21; 21, 22; 22, 23;
                           16, 24; 24, 25; 25, 26; 26, 27; 27, 28; 28, 29; 29, 30];
intrinsicSlingMuscleIdx_reduced = [
    0 1; 1 2; 3 4; 4 5; 6 7; 7 8
];

writematrix(intrinsicSlingMuscleIdx_reduced, '../../intrinsic_sling_muscle_idx_reduced.csv');
writematrix(intrinsicSlingMuscleIdx, '../../intrinsic_sling_muscle_idx.csv');