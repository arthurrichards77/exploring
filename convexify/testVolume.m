% initial polytope Px<=q
P = [1 0; 0 1; -1 0; 0 -1; 1 1; -1 -1; 1 -1; -1 1];
q = [1;1;1;1;1.4;1.4;1.4;1.4];

% approximate circle
P = [cos(2*pi*(1:10)/10)' sin(2*pi*(1:10)/10)'];
q = ones(10,1);

% numbers of test points
nums = floor(logspace(1,6,20));
VS = [];

% loop
for ii=1:numel(nums),

    % test volume
    Vs(ii) = polyVol(P,q,nums(ii));
    
end

figure
semilogx(nums,Vs)