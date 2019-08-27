function [qFree,flag,qFreeInit] = convexify(P,q,ptsIn,ptsOut,ptsId,opts)
%#codegen
%
%   [qFree,flag] = convexify(P,q,ptsIn,ptsOut,ptsId,opts)
%
% Find a polytope Px<=qFree that is a subset of Px<=q, contains the points
% ptsIn, but does not overlap with a set of convex obstacles
%
% If ptsId is omitted, ptsOut is interpreted as a set of discrete points
% that must all be excluded from the chosen polytope
%
% If included, ptsId identifies the obstacle to which each point belongs,
% e.g. if the first five points in ptsOut are the vertices of one convex
% obstacle, then the first five entries in ptsId should all be the same.
%
% Structure "opts" contains optional settings.
%   opts.scoring controls the volume maximization weights
%
% Returns flag=0 if none can be found and flag=1 if it can
%

% start by finding minimum setting for each direction to that ptsIn are in
qMin = max(P*ptsIn,[],2);

% check each outside point against directions P
qOut = P*ptsOut;

% number of obstacle points
nOut = size(ptsOut,2);

% assume each point independent if no "ptsId" vector provided
if ~exist('ptsId'),
    ptsId = 1:nOut;
    obstId = 1:nOut;
    nObs = nOut;
else,
    if isempty(ptsId),
        ptsId = 1:nOut;
        obstId = 1:nOut;
        nObs = nOut;
    else,
        obstId = unique(ptsId);
        nObs = numel(obstId);
    end
end

% default option structure
if ~exist('opts'),
    opts.hello = 1; % to avoid empty structure issues
end

% initialize allocation of obstacles to rays
obsAlloc = zeros(1,nObs);
% and which obstacle limits each ray
rayLimit = zeros(1,length(qMin));

% check if any obstacle points are inside the minimal polygon
if any(all(qOut<=repmat(qMin,1,nOut))),
    flag = 0;
    qFree = qMin;
    qFreeInit = qMin;
    warning('convexify:pointinpoly','Obstacle point inside the hull of included points: no suitable polygon exists')
    return
end

% OK - polygon must exist
flag = 1;

% start with all sides furthest out
qFree = q;
qFreeInit = q;

% work through each obstacle
for ob = obstId,
    
    % margins (directions along each ray) for this point
    qThis(:,ob) = min(qOut(:,ptsId==ob),[],2);
    
    % take the ones that dont encroach on the inner poly
    testq1 = qThis(:,ob)<qMin;
    
    % problem if they all do
    if all(testq1),
        flag = 0;
        qFree = qMin;
        warning('convexify:nohalf','No suitable choice of halfspace: no suitable polygon exists')
        return
    end
    
    % find distances inside current poly
    testq2 = qThis(:,ob)-qFree;
    
    % knock out ones that would encroach on inner poly
    testq2(testq1) = -inf;
    
    % find least encroaching
    [dq,pk]=max(testq2);
    if dq<0,
        qFree(pk) = qFree(pk)+dq;
        rayLimit(pk) = ob;
    end
    obsAlloc(ob) = pk;
    
end

%% try to enlarge volume

% grab the current polygon
qFreeInit = qFree;

% start with set of test points for volume estimation
% default option for volume weighting
if ~isfield(opts,'numrandpts'),
    opts.numrandpts = 10*nObs;
end

numTests = opts.numrandpts;
testPts = rand(2,numTests)*2 - 1;

% shift and scale points
xbar = P\q;
scl = max(abs(P*xbar - q));
testPts(1,:) = xbar(1)+testPts(1,:)*scl;
testPts(2,:) = xbar(2)+testPts(2,:)*scl;

% default option for volume weighting
if ~isfield(opts,'scoring'),
    opts.scoring = 1;
end

% points can have different scores
switch opts.scoring
    case 2
        ptScores = sqrt(sum(testPts.*testPts)); % favour points far from origin
    case 3
        ptScores = 1./(1+sum(testPts.*testPts)); % favour points closer to origin
    case 4
        ptScores = ones(1,numTests) + testPts(1,:); % favour points in +x direction
    otherwise
        ptScores = ones(1,numTests); % all points equal
end

% initial test
testRes = (P*testPts <= repmat(qFree,1,numTests));

% volume estimate
vol = sum(ptScores.*all(testRes,1));

% SA temperature
saTemp = vol;

% default option for volume weighting
if ~isfield(opts,'saiters'),
    opts.saiters = 20*nObs;
end

% SA optimizer loop
for kk=1:opts.saiters,
    
    % choose a ray at random
    thisRay = ceil(rand(1)*length(qFree));
    
    % alternative - work through rays
    %thisRay = 1+mod(kk,length(qFree));
    
    % and the obstacle pinning it
    thisObs = rayLimit(thisRay);
    
    if thisObs>0,
        
        % find candidate for swapping
        rayOpts = (qThis(:,thisObs)>=qMin);
        rayOpts(thisRay) = false;
        
        % check if there are any
        numOpts = sum(rayOpts);
        if numOpts>0,
            % choose one at random
            takeOpt = ceil(numOpts*rand(1));
            rayList = find(rayOpts);
            newRay = rayList(takeOpt); % so thisObs is reassigned to newRay
            
            % candidate setup
            qFreeCand = qFree;
            rayLimitCand = rayLimit;
            obsAllocCand = obsAlloc;
            
            % reallocate obstacle to newly chosen ray
            obsAllocCand(thisObs) = newRay;
            
            % set the new ray
            if qThis(newRay,thisObs)<qFree(newRay),
                qFreeCand(newRay) = qThis(newRay,thisObs);
                rayLimitCand(newRay) = thisObs;
            end
            
            % list the obstacles still associated with the old ray
            remObs = find(obsAllocCand==thisRay);
            
            % use the outer limit if list is empty
            if isempty(remObs),
                qFreeCand(thisRay) = q(thisRay);
                rayLimitCand(thisRay) = 0;
            else,
                % find the new limit
                [qLim,iLim] = min(qThis(thisRay,remObs));
                if qLim<q(thisRay),
                    % adjust the ray and its limit to new obstacle
                    qFreeCand(thisRay) = qLim;
                    rayLimitCand(thisRay) = remObs(iLim);
                else
                    % set to outer limit
                    qFreeCand(thisRay) = q(thisRay);
                    rayLimitCand(thisRay) = 0;
                end
            end
            % new volume estimate
            testResCand = (P*testPts <= repmat(qFreeCand,1,numTests));
            volCand = sum(ptScores.*all(testResCand,1));
            
            % SA temperature cooling
            saTemp = saTemp*0.97;
            
            % proability of acceptance
            saProbAcc = exp((volCand-vol)/saTemp);
            
            % accept it if better
            if saProbAcc>=rand(1),
                vol = volCand;
                testRes = testResCand;
                qFree = qFreeCand;
                rayLimit = rayLimitCand;
                obsAlloc = obsAllocCand;
            end
            
        end
        
    end
    
end