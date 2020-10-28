function [foot1, foot2] = getCommonFootOfPerpendicularLine(s1, e1, s2, e2)
%getCommonFootOfPerpendicularLine gets calculates the foots of the common
%perpendicular line of two non-coplanar lines.

a1 = s1(1); a2 = s1(2); a3 = s1(3);
b1 = e1(1); b2 = e1(2); b3 = e1(3);
c1 = s2(1); c2 = s2(2); c3 = s2(3);
d1 = e2(1); d2 = e2(2); d3 = e2(3);

% define
% p = foot1 = a + k1*(a-b);
% q = foot2 = c + k2*(c-d);

% set up equations:
% |ab|.|pq| = 0
% |cd|.|pq| = 0
% simplify we have:
% alpha1*k1 + beta1*k2 + gamma1 = 0
% alpha2*k1 + beta2*k2 + gamma2 = 0
% where:
alpha1 = + ((a1-b1)^2 + (a2-b2)^2 + (a3-b3)^2);
beta1  = - ((a1-b1)*(c1-d1) + (a2-b2)*(c2-d2) + (a3-b3)*(c3-d3));
gamma1 = (a1-b1)*(a1-c1) + (a2-b2)*(a2-c2) + (a3-b3)*(a3-c3);

alpha2 = + ((a1-b1)*(c1-d1) + (a2-b2)*(c2-d2) + (a3-b3)*(c3-d3));
beta2  = - ((c1-d1)^2 + (c2-d2)^2 + (c3-d3)^2);
gamma2 = (a1-c1)*(c1-d1) + (a2-c2)*(c2-d2) + (a3-c3)*(c3-d3);

% solve
k = -inv([alpha1, beta1; alpha2, beta2])*[gamma1; gamma2];

% then
foot1 = [a1, a2, a3] + k(1)*([a1, a2, a3]-[b1, b2, b3]);
foot2 = [c1, c2, c3] + k(2)*([c1, c2, c3]-[d1, d2, d3]);





end