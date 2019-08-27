function V = polyVol(P,q,numPts)
%
% estimate volume of polytope P*x<=q
%

% number of points, if not specified
if ~exist('numPts'),
    % number of random sampling points
    numPts = 1000;
end

% assume bounds for now
plo = [-1;-1];
phi = [1;1];

% volume of bounding box
Vbox = prod(abs(phi-plo));


% generate points
testPts = repmat(plo,1,numPts) + repmat(phi-plo,1,numPts).*rand(numel(plo),numPts);

% test points
testRes = (P*testPts <= repmat(q,1,numPts));

% find those inside poly
inPoly = all(testRes,1);

% how many?
numInPoly = sum(inPoly);

% volume
V = Vbox*numInPoly/numPts;