clear; close all;

pos = load('../test.csv');
dx = pos(2:end) - pos(1:(end-1));

plot(pos);