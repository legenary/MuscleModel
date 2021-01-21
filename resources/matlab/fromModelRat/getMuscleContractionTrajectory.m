clear;

%% intrinsic sling muscle
totalTime = 10;
startTime = 0.5;
contractTime = 2;
fps = 60;
contractRatio = 0.5;

T = contractTime*2;
t = 1/fps:(1/fps):contractTime;
trajectory = contractRatio*ones(totalTime*fps, 1);
trajectory(1:startTime*fps) = ones(startTime*fps, 1);
trajectory(startTime*fps+(1:contractTime*fps)) = cos(2*pi/T*t)*(1-contractRatio)/2 + (1+contractRatio)/2;

% plot(trajectory);
writematrix(trajectory, '../../intrinsic_sling_muscle_contraction_trajectory.csv')

%% extrinsic nasolabialis muscle
totalTime = 10;
startTime = 0.5;
contractTime = 2;
fps = 60;
contractRatio = 0.8;

T = contractTime*2;
t = 1/fps:(1/fps):contractTime;
trajectory = contractRatio*ones(totalTime*fps, 1);
trajectory(1:startTime*fps) = ones(startTime*fps, 1);
trajectory(startTime*fps+(1:contractTime*fps)) = cos(2*pi/T*t)*(1-contractRatio)/2 + (1+contractRatio)/2;

% plot(trajectory);
writematrix(trajectory, '../../nasolabialis_contraction_trajectory.csv')

%% extrinsic maxillolabialis muscle
totalTime = 10;
startTime = 0.5;
contractTime = 2;
fps = 60;
contractRatio = 0.8;

T = contractTime*2;
t = 1/fps:(1/fps):contractTime;
trajectory = contractRatio*ones(totalTime*fps, 1);
trajectory(1:startTime*fps) = ones(startTime*fps, 1);
trajectory(startTime*fps+(1:contractTime*fps)) = cos(2*pi/T*t)*(1-contractRatio)/2 + (1+contractRatio)/2;

% plot(trajectory);
writematrix(trajectory, '../../maxillolabialis_contraction_trajectory.csv')