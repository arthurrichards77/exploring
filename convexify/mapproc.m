close all
clear all

%% load raw image
imRaw = imread('57638981.png');
imRaw = imread('newmap.bmp');
%subplot 211
%imshow(imraw)

% extract the walls by colour
occ = (imRaw(:,:,1)>imRaw(:,:,3)); % they're red

% pixel size
pixelSize = 0.1;

% make reference object
occRef = imref2d(size(occ),pixelSize,pixelSize);

% and get the box
occBox = [occRef.XWorldLimits occRef.YWorldLimits];

% show the map
figure
%imshow(occ,occRef)
imshow(imRaw,occRef)

%% convert to 1D for Simulink plotting
occVec = [size(occ)';occBox';reshape(occ,numel(occ),1)];

%% find the goal marker pixels
goalPix = (imRaw(:,:,2)>imRaw(:,:,3)); % they're green

% find the mean of the goal marker cells
[r,c]=find(goalPix);
rCentroid = mean(r);
cCentroid = mean(c);

% determine that in real world coords
[xGoal,yGoal] = convOccToWorld(cCentroid,rCentroid,size(occ),occBox);

% plot it on the image
hold on
plot(xGoal,yGoal,'gs')

%% ray trace

% ray origin
cx = 38;
cy = 45;

% max range
rmax = 50;

% angle range
thetaRng = -pi/2+linspace(-pi/4,pi/4,101);

[rScan,fScan,xScan,yScan] = rangeScan(cx,cy,occ,occBox,thetaRng,rmax);


% plot back on world
hold on
plot(xScan,yScan,'k.', ...
     [repmat(cx,numel(thetaRng),1) xScan']', [repmat(cy,numel(thetaRng),1) yScan']', 'g-')
 
 %% convexifying
 
 % initial polytope Px<=q
P = [1 0; 0 1; -1 0; 0 -1; 1 1; -1 -1; 1 -1; -1 1];
q = 40*[1;1;1;1;1.4*0;1.4;1.4;1.4*0];
q = q + P*[cx;cy];

% plot initial
h=plotPoly(P,q,'k--');
    
% points required to be inside
ptsIn = [cx + [0 0 0];
         cy + [0 -5 -10]];

%ptsIn = [0;0];

% options
opts.saiters = 2000;

[qFree,flag,qF2] = convexify(P,q,ptsIn,[xScan; yScan],[],opts);

h=plotPoly(P,qFree,'m');

% plot inside points
    plot(ptsIn(1,:),ptsIn(2,:),'-b+')

