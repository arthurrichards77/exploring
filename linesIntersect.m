function [flag,intersectPoint] = linesIntersect(a1,a2,b1,b2)
%#codegen
%
% [flag,intersectPoint] = linesIntersect(a1,a2,b1,b2)
%
% all inputs should be 2 x 1 vectors
%
% flag = 2 if they strictly intersect
%        1 if they intersect at an endpoint
%        0 otherwise
%
% check if a1-a2 and b1-b2 intersect
%

% tolerance
tol = 1e-9;

% quick precheck based on circumcircle
if norm(0.5*(a1+a2-b1-b2)) >= tol + 0.5*(norm(a1-a2)+norm(b1-b2)),
  intersectPoint =[];
  flag = 0;
  return
end

X1 = a1(1);
Y1 = a1(2);
X2 = a2(1);
Y2 = a2(2);
A1 = b1(1);
B1 = b1(2);
A2 = b2(1);
B2 = b2(2);

dx = X2 - X1;
dy = Y2 - Y1;
da = A2 - A1;
db = B2 - B1;

if (da * dy - db * dx) == 0,
  intersectPoint =[];
  flag = 0;
else,
  lam = (dx * (B1 - Y1) + dy * (X1 - A1)) / (da * dy - db * dx);
  gam = (da * (Y1 - B1) + db * (A1 - X1)) / (db * dx - da * dy);
  % test 
  if (lam>=tol)&&(lam<=1-tol)&&(gam>=tol)&&(gam<=1-tol),
    flag = 2;
    intersectPoint = gam*a2 + (1-gam)*a1;
  elseif (lam>=0)&&(lam<=1)&&(gam>=0)&&(gam<=1),
    flag = 1;
    intersectPoint = gam*a2 + (1-gam)*a1;
  else
    intersectPoint =[];
    flag = 0;
  end
end
