function y=visualizeSim(u)
%
% visualizer add-on for exploration simulator
%

% separate inputs - current state
x = u(1:6);

% current setpoint
ys = u(7:8);

% predictions
px = u(9:24);
py = u(25:40);

%goal
goal = u(41:42);

% vertices of current op region
verts = u(43:46);

% obstacles
obstIn = u(47:end);
% number of obstacles
nObst = floor(length(obstIn)/4);
% reformat
obst = reshape(obstIn,4,nObst)';

figure(1)
plot(x(1),x(4),'bo',px,py,'b-',ys(1),ys(2),'k+', ... 
     goal(1),goal(2),'gx',obst(:,[1 2 2 1 1])',obst(:,[3 3 4 4 3])','r', ...
     verts([1 2 2 1 1]),verts([3 3 4 4 3]),'g-', ...
     [x(1) ys(1)],[x(4) ys(2)],'m:')
axis([-6 6 -6 6])
axis square

% token output
y=0;