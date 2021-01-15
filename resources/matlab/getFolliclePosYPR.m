
% at the end of this script, vec_top2D means the basepoint location, which is the top end of the follicle,
% and vec_bot2D means the deeper end of the follicle.
% 

clear;
ca;

load('NewRatMapModelOutput.mat');

%% Fit a plane to basepoints, get orientation matrix here
P3D = modelPointsBP(1:35, :);
P2D = [zeros(size(P3D, 1), 1), modelPointsBP(1:35, 2:3)];
[~, P2Din3D, T] = procrustes(P3D, P2D, 'reflection', 0, 'scaling', 0);
% Note: P2D = T.T*(P3D'-T.c')



%% get emergence vectors in world reference frame
dir_0 = [0, -1, 0]';
l = 1.4;
dir = zeros(35, 3);
for w = 1:35
    dir(w, :) = rotz(modelThetaW(w), 'deg')*...
                rotx(modelPhiW(w), 'deg')'*...
                roty(modelZetaW(w), 'deg')* dir_0;
end
vec_top3D = P3D;
vec_bot3D = P3D - l*dir;

vec_top2D = (T.T*(vec_top3D'-T.c'))';
vec_bot2D = (T.T*(vec_bot3D'-T.c'))';

vec_top2D = vec_top2D(:, [2, 3, 1]);
vec_bot2D = vec_bot2D(:, [2, 3, 1]);
vec_top2D(:, 1) = -vec_top2D(:, 1);
vec_bot2D(:, 1) = -vec_bot2D(:, 1);

figure('Color', 'w'); hold on;
for i = 1:35
    plot3d([vec_top2D(i,:); vec_bot2D(i,:)], 'k-');
    plot3d(vec_top2D(i,:), 'ro');
%     plot3d(vec_bot2D(i,:), 'bo');
end
axis equal
xlabel('x');
ylabel('y');
zlabel('z');

%% get emergence angles in face reference frame

%  ^ y
%  |
%  |     E6 E5 E4 E3 E2 E1
%  |     D6 D5 D4 D3 D2 D1
%  |     C6 C5 C4 C3 C2 C1
%  |        B5 B4 B3 B2 B1
%  |        A5 A4 A3 A2 A1
%  +--------------------------> x

direction_vector = vec_top2D - vec_bot2D;

follicle_pos_ypr = zeros(35, 6);
follicle_pos_ypr(:, 1:3) = (vec_top2D + vec_bot2D)/2;

for i = 1:35
    [th, pi, r] = cart2sph(direction_vector(i, 1), direction_vector(i, 2), direction_vector(i, 3));
    follicle_pos_ypr(i, 4:6) = [th, pi, 0];
end

writematrix(follicle_pos_ypr, '../follicle_pos_ypr.csv')
save('follicle_pos', 'vec_top2D', 'vec_bot2D');


    
    
    
    
    
    


