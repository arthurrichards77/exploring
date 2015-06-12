function y=visualizeSim(u)
%
% visualizer add-on for exploration simulator
%

% separate inputs - current state
x = u(1:6);

% current setpoint
ys = u(7:8);

% predictions
sol = u(9:168);
px = sol(3:10:end);
py = sol(7:10:end);

%goal
goal = u(169:170);

% vertices of current op region
verts = u(171:174);

% obstacles
obstIn = u(175:end);
% number of obstacles
nObst = floor(length(obstIn)/4);
% reformat
obst = reshape(obstIn,4,nObst)';

% do the plot
f=figure(1);
plot(x(1),x(4),'bo',px,py,'b-',ys(1),ys(2),'k+', ... 
     goal(1),goal(2),'gx',obst(:,[1 2 2 1 1])',obst(:,[3 3 4 4 3])','r', ...
     verts([1 2 2 1 1]),verts([3 3 4 4 3]),'g-', ...
     [x(1) ys(1)],[x(4) ys(2)],'m:')
axis([-6 6 -6 6])
axis square

% see if figure doesn't yet have a tickbox
if(isempty(get(f,'UserData'))),
    % add it and store handle in this useful little property holder
    ht=uicontrol('Style','CheckBox','String','Continue','Value',1);
    set(f,'UserData',ht);
end

% set output to stop sim if checkbox unticked
y=get(get(f,'UserData'),'Value');