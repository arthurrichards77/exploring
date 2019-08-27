function [ptsOut,numSegs] = subdivLineSegs(ptsIn,maxLen)
%
% assume adjacent columns in ptsIn are line segment end points
% subdivide until none shorter than maxLen

% number of segments
numSegs = floor(size(ptsIn,2)/2);

% truncate, just in case of stupidly having odd number of points
ptsOut = ptsIn(:,1:(2*numSegs));

% segment counter
thisSeg = 1;

% loop has upper limit in case of runaway
for ii=1:1000,
    
    % terminate if no new segments to test
    if thisSeg>numSegs,
        break
    end
    
    % get segment endpoints
    thisP1 = ptsOut(:,2*thisSeg-1);
    thisP2 = ptsOut(:,2*thisSeg);
    
    % check length
    if norm(thisP1-thisP2)<=maxLen,
        % pass - move to next segment
        thisSeg = thisSeg + 1;
    else,
        % too long - divide and recheck
        newP = 0.5*(thisP1+thisP2);
        % first new segment is one end
        ptsOut(:,2*thisSeg) = newP;
        % second new segment is remainder
        ptsOut(:,2*numSegs+[1:2]) = [newP thisP2];
        % increment segment total
        numSegs = numSegs+1;
    end
end
