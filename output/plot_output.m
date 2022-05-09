clear; ca;

M = csvread('test_output.csv');

figure; hold on;
T = (0:(1/60):100)';

t = T(1:length(M));
S = M(:,1);
plot(t, S);


