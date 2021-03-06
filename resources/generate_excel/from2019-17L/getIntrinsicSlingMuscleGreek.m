clear; close all;
load('follicle_pos');


figure; hold on;
plot3d(vec_top2D, 'ko');
plot3d(vec_top2D([1, 6, 11, 17], :), 'ro');

plot3d(vec_bot2D, 'ko');
plot3d(vec_bot2D([1, 6, 11, 17], :), 'ro');

for i = 1:31
    plot3d([vec_bot2D(i,:); vec_top2D(i,:)], 'k-');
end

for i = [1, 6, 11, 17]
    plot3d([vec_bot2D(i,:); vec_top2D(i,:)], 'r-');
end

% intrinsic muslce is 70%
intrinsic_muscle_index = [0	1
1	2
2	3
3	4
0	6
5	6
6	7
7	8
8	9
5	11
10	11
11	12
12	13
13	14
14	15
10	17
16	17
17	18
18	19
19	20
20	21
21	22
22	23
16	24
24	25
25	26
26	27
27	28
28	29
29	30
]+1;

for i = 1:length(intrinsic_muscle_index)
    first_idx = intrinsic_muscle_index(i,1);
    second_idx = intrinsic_muscle_index(i,2);
    first_point = vec_top2D(first_idx, :);
    second_point = vec_bot2D(second_idx, :) * 0.7 + vec_top2D(second_idx, :) * 0.3;
    plot3d([first_point; second_point], 'b-');
end

axis equal



%% now create greek insertion
intrinsicSlingMuscleGreek = zeros(4, 4);
intrinsicSlingMuscleGreek(:,1) = [0; 5; 10; 16];

tempNode = ((vec_top2D(1,:)*2-vec_top2D(2,:))...
           +(vec_top2D(1,:)*2-vec_top2D(7,:)))...
           /2;
intrinsicSlingMuscleGreek(1, 2:4) = tempNode;

tempNode = ((vec_top2D(6,:)*2-vec_top2D(7,:))...
           +(vec_top2D(6,:)*2-vec_top2D(12,:)))...
           /2;
intrinsicSlingMuscleGreek(2, 2:4) = tempNode;

tempNode = ((vec_top2D(11,:)*2-vec_top2D(12,:))...
           +(vec_top2D(11,:)*2-vec_top2D(18,:)))...
           /2;
intrinsicSlingMuscleGreek(3, 2:4) = tempNode;

tempNode = ((vec_top2D(17,:)*2-vec_top2D(18,:))...
           +(vec_top2D(17,:)*2-vec_top2D(25,:)))...
           /2;
intrinsicSlingMuscleGreek(4, 2:4) = tempNode;

plot3d(intrinsicSlingMuscleGreek(:, 2:4), 'ro');

% writematrix(intrinsicSlingMuscleGreek, '../../intrinsic_sling_muscle_greek.csv')

