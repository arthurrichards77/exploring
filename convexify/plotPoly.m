function h=plotPoly(P,q,col)
%
% plot polygon Px<=q
% in colour col
% 2D only

% check size
if size(P,2)~=2,
    error('plotPoly:not2D','P must have 2 columns for 2D polygon')
end

% estimate outer bound
M = 2*max(abs(q)./sum(abs(P),2));

% find vertices
Xs = vertices(P,q,M);

% choose patch or plot depending on col
if length(col)>1,
    % plot it
    h = plot([Xs(:,1); Xs(1,1)],[Xs(:,2); Xs(1,2)],col);
else,
    % patch it
    h = patch(Xs(:,1),Xs(:,2),col);
end

function [Xs] = vertices(A,B,M)
%
% Xs = vertices(A,B,M)
%
% find vertices of Ax<=B, |x|<M, (2-D only)
%
% 2-D only!!!!!!

% set up initial vertex list
Xs_tmp = M*[-1 -1 1 1;
        -1 1 -1 1]';

% work through each constraint
for ii=[1:(size(A,1)-1)],
  
  % work through all lower constraints
  for jj=[(1+ii):size(A,1)],
  
    % find intersection
    Aint = [A(ii,:); A(jj,:)];
    if rank(Aint)>1,

      % lines do intersect
      x = Aint \ [B(ii,:); B(jj,:)];
    
      % add vertex to new vertex list
      Xs_tmp = [Xs_tmp; x'];
    end
    
  end % continue finding vertices
  
end

%keyboard

% now remove all those outside
ind = find(max(A*Xs_tmp'-B*ones(1,size(Xs_tmp,1)))<=1e-6);
Xs = Xs_tmp(ind,:);

% bail if no points
if min(size(Xs))<2,
  return
end

% re-order for plotting
% first find centroid
xc = mean(Xs);

%xc
%Xs

% angles around centroid
angs = atan2(Xs(:,1) - xc(1),Xs(:,2) - xc(2));

% sort angles
[asort,isort] = sort(angs);

% sort vertices
Xs = Xs(isort,:);

%keyboard