close all
clear all

% initial polytope Px<=q
P = [1 0; 0 1; -1 0; 0 -1; 1 1; -1 -1; 1 -1; -1 1];
q = [1;1;1;1;1.4;1.4;1.4;1.4];



% points required to be inside
ptsIn = [0 0.2 0.4;
    0 0.1 -0.2];

%ptsIn = [0;0];

% points required to be outside
% random triangles
ptsOut = 1*2*(rand(2,12)-0.5);
ptsId = [1 1 1 2 2 2 3 3 3 4 4 4];
% random points
ptsOut = 1*2*(rand(2,50)-0.5);
ptsId = 1:size(ptsOut,2);
% random line segments
%ptsOut = 1*2*(rand(2,8)-0.5);
%ptsOut = [-0.255374678440976 0.65906564905303 -0.254931520200921 0.745105129295117 0.336928548722751 0.307701184125176 -0.186546169821272 0.867451319091859 -0.0309034563300192 -0.165905092514466 0.975949402461664 -0.222232448175143;0.874269332683125 0.69817095990891 0.186369150436981 0.867003217014211 -0.58644708412979 -0.8558968975587 0.333863066414997 0.62190006447653 0.513498420131024 0.943571985978588 0.728295058062405 -0.0905163439217764];
%ptsId = kron((1:6),[1 1]);

% subdivide long lines
%[ptsOut,numSegs] = subdivLineSegs(ptsOut,0.1);
%ptsId = kron((1:numSegs),[1 1]);

% try different objectives
for mm=1:4,
    opts.scoring = mm;
    
    subplot(2,2,mm)
    % plot initial
    h=plotPoly(P,q,'g');
    hold on
    
    % convexify
    [qFree,flag,qF2] = convexify(P,q,ptsIn,ptsOut,ptsId,opts);
    
    % plot poly
    h=plotPoly(P,qFree,'m');
    h=plotPoly(P,qF2,'r--');
    
    % plot obs
    plot(ptsOut(1,:),ptsOut(2,:),'kx')
    for ob = unique(ptsId),
        ps = ptsOut(:,ptsId==ob);
        patch(ps(1,:),ps(2,:),'r')
    end
    
    % plot inside points
    plot(ptsIn(1,:),ptsIn(2,:),'-b+')
    
end