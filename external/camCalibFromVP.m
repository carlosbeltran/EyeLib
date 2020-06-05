% From https://github.com/andzaytsev/pt

function [f, u0, v0, R] = camCalibFromVP(VP)
% === Input === 
% VP: [2 x 3] vanishing points 
%
% === Output === 
% f - focol length
% u0, v0 - principle center
% R - rotation matrix

x1 = VP(1,1);   y1 = VP(2,1);
x2 = VP(1,2);   y2 = VP(2,2);
x3 = VP(1,3);   y3 = VP(2,3);

% Compute u0, v0
x23 = x2 - x3;  y23 = y2 - y3;
x12 = x1 - x2;  y12 = y1 - y2;

A = [x23, y23; x12, y12];
b = [x1*x23 + y1*y23; x3*x12 + y3*y12];
x = A\b;
u0 = x(1); v0 = x(2);

% Compute focal length f
f = abs(sqrt(-(x1 - u0)*(x2 - u0) - (y1 - v0)*(y2 - v0)));

% Solve for rotation matrix
K = [f, 0, u0; 0, f, v0; 0, 0, 1];
vpH = cat(1, VP, ones(1,3));
R = K\vpH;

end

