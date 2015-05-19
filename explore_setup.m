% double integrator MPC setup

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% START OF MPC PROBLEM SET UP
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all
addpath('simulink_mpc/ks_solver')

% time step
dt = 0.15;

%% system matrices
% Horizontal:
% continuous transfer function from command to velocity
[Ah Bh Ch] = tf2ss(12,[0.35 2 1]);
Ah = rot90(rot90(Ah));
Bh = rot90(rot90(Bh))*Ch(end);
Ch(end) = Ch(end)/Ch(end);
Ch = rot90(rot90(Ch));
% C = eye(3);

% augment to include pos as state 1
Ah = [0 1 0; zeros(2,1) Ah];
Bh = [0; Bh];

% measurement
Ch = [1 0 0];

% Discrete system matrices
[Ad,Bd,Cd,Dd] = c2dt(Ah,Bh,Ch,dt,0);

% Rename
A = Ad;
B = Bd;

% Vertical:
% continuous transfer function from command to velocity
[Av Bv Cv] = tf2ss(0.5,[0.08 0.05 1]);
Av = rot90(rot90(Av));
Bv = rot90(rot90(Bv))*Cv(end);
Cv(end) = Cv(end)/Cv(end);
Cv = rot90(rot90(Cv));

% augment to include pos as state 1
Av = [0 1 0; zeros(2,1) Av];
Bv = [0; Bv];

% measurement
Cv = [1 0 0];


% Rotational:
% continuous transfer function from command to yaw rate
[Aw Bw Cw] = tf2ss(1.8,[0.15 1]);
Bw = Bw*Cw;
Cw = Cw/Cw;
% [Adw,Bdw,Cdw] = c2dt(Aw,Bw,Cw,dt,0);

% Augment to include rotation as state 1
Aw = [0 1; 0 Aw];
Bw = [0; Bw*Cw];
Cw = [1 0];

% % Combine lateral and rotations
Alat = zeros(8,8);
Alat(1:3:end,1:3:end) = Ah;
Alat(2:3:end,2:3:end) = Ah;
Alat(3:3:end,3:3:end) = Aw;
Blat = zeros(8,1);
Blat(1:3:end) = Bh;
Blat(2:3:end) = Bh;
Blat(3:3:end) = Bw;
Clat = zeros(1,8);
Clat(1:3:end) = Ch;
Clat(2:3:end) = Ch;
Clat(3:3:end) = Cw;

%% Estimator matrices - Lateral
Ae = Ah;
Be = Bh;
Ce = Ch;

% pole placement
Le = place(Ae', Ce', -[3.1 3.0 3.2])';

%% Estimator matrices - vertical
Lv = place(Av', Cv', -[3.1 3.0 3.2])';

%% Estimator matrices - rotational
Lw = place(Aw', Cw', -[3.1 3.0])';

%% start of MPC setup

% Discrete system matrices
[Ad,Bd,Cd,Dd] = c2dt(Ah,Bh,Ch,dt,0);

% A = [1 dt 0.5*dt*dt; 
%      0 1 dt;
%      0 0 1]; % states [pos; vel; acc]
% B = [0.5*dt*dt  ;
%      dt         ;
%      1          ]; % inputs [delta-acc]

A = Ad;
B = Bd;

% Turn into delta u control
A = [A B; zeros(1,size(A,2)) ones(1,size(B,2))];
B = [B; ones(1,size(B,2))];

% Change such that both axes are included in MPC
A = kron(eye(2),A);
B = kron(eye(2),B);

% hard constraints are Fx*x+Fu*u<=Ymax
% Fx = [0 0 0 0;
%      0 0 0 0];
% Fu = [1 ;
%     -1 ];
% f = [0.5 % move <= 0.5
%      0.5 ]; 
Fx = [0 0 0 0;
      0 0 0 0];
Fu = [ 1 ;
      -1 ];
f = [0.2 % move <= 0.5
     0.2 ]; 
 
% Change such that both axes are included in MPC
Fx = kron(eye(2),Fx);
Fu = kron(eye(2), Fu);
f = kron([1 1]', f);
Fp = zeros(length(f),4); % no parameter involvement in hard constraints

% soft constraint weight
softWeight = 100;
% soft constraints: cost includes sum_i ( max{Cs_i*x + Ds_i*u - Ysmax, 0} )
Fxs = [-1  0  0  0; 
        1  0  0  0;
        0  1  0  0; 
        0 -1  0  0;
        0  0  0  1;
        0  0  0 -1]*softWeight;
Fus = [0 ;
       0 ;
       0 ;
       0 ;
       0 ;
       0 ]*softWeight;
fs = [0; % posn <= 0, to be set by parameter
      0;
      0.5; % vel <= 0.2
      0.5;
      0.35; % u <= 0.37
      0.35 ]*softWeight; 
 
% Change such that both axes are included in MPC
Fxs = kron(eye(2),Fxs);
Fus = kron(eye(2), Fus);
fs = kron([1 1]', fs);

% parameter plim is [xmin xmax ymin ymax]
Fps = [1 0 0 0; % first line is  xmin-x<=0
       0 -1 0 0; % next x-xmax<=0
       zeros(4,4);
       0 0 1 0; % ymin-y<=0
       0 0 0 -1; % y-ymax<=0
       zeros(4,4)]*softWeight;

% terminal constraints Ff*x(N)<=Ff;
Fxf = [-1  0  0  0; 
        1  0  0  0]; % constraint only on position
ff = zeros(2,1);  % limits set by parameters

% Change such that both axes are included in MPC
Fxf = kron(eye(2),Fxf);
ff = kron([1 1]', ff);

% parameter depends again on plim
Fpf = diag([1 -1 1 -1]);

% OVERWRITE terminal hard constraints playing up
Fxf = 1e-6*eye(8); % effectively no constraint at all
ff = ones(8,1);
Fpf = zeros(8,4);

% terminal equality constraints Ef*x(N)==ef
Ef = eye(8)-A;
Ef = Ef([1:2 5:6],:);
%Ef = [0 1 0; 0 0 1];
ef = [0;0;0;0];
Ed = zeros(4,8); % no disturbance of eqs on disturbance

% cost 
% xN*Qf*xN + qf'*xN + sum (x'*Q*x + u'*R*u + q'*x + r'*u)
Qf = diag([5 5 5 2 5 5 5 2]);
Q = diag([5 5 5 2 5 5 5 2]);
R = diag([10 10]);
qf = [0; 0; 0; 0; 0; 0; 0; 0];
q = [0; 0; 0; 0; 0; 0; 0; 0];
r = [0; 0]; 

% horizon
T = 16;

%% build QP matrices

% see help information in BUILDMATRICES for details of QP matrix format
[H,g,gt,P,hx,hc,C,bx,bd,Ps,hxs,hcs,Ef_block,Ed_block,ef_block,hp,hps] = buildmatrices(A,B,Q,R,q,r,Qf,qf,T,Fx,Fu,f,Fxs,Fus,fs,Fxf,ff,Ef,ef,Ed,Fp,Fps,Fpf);

% %% checking problem setup
% 
% % sizes
% n = size(A,1);
% m = size(B,2);
% ell = size(Fx,1);
% ellf = size(Ff,1);
% ellef = size(Ef,1);
% ells = size(Fxs,1);
% 
% % checks
% if size(B,1)~=n,
%     error('B must have same number of rows as A')
% end
% if size(A,2)~=n,
%     error('A must be square')
% end
% if size(Q,2)~=n,
%     error('Q must be the same size as A')
% end
% if size(Q,1)~=n,
%     error('Q must be the same size as A')
% end
% if size(Fx,2)~=n,
%     error('Fx must have same number of columns as A')
% end
% if size(Fu,1)~=ell,
%     error('Fu must have same number of rows as Fx')
% end
% if size(Fu,2)~=m,
%     error('Fu must have same number of columns as B')
% end
% if size(Fx,2)~=n,
%     error('Fx must have same number of columns as A')
% end
% if size(Ff,2)~=n,
%     error('Ff must have same number of columns as A')
% end
% if size(ff,1)~=ellf,
%     error('ff must have same number of rows as Ff')
% end
% if size(Ef,2)~=n,
%     error('Ef must have same number of columns as A')
% end
% if size(ef,1)~=ellef,
%     error('ef must have same number of rows as Ef')
% end
% 
% %% compile MPC problem
% 
% % min z'*H*z + (gt*xt+g)'*z + sum(max(0,Ps*z-hs))
% % s.t. P*z <= h
% %      C*z == b
% %
% % where h = hx*x0 + hc
% %      hs = hxs*x0 + hcs + hcp*plim
% % and b = bx*x0 + bd*d
% %
% % where plim is [xmin xmax ymin ymax]
% 
% H = blkdiag(R,kron(eye(T-1),blkdiag(Q,R)),Qf);
% P = blkdiag(Fu,kron(eye(T-1),[Fx Fu]),Ff);
% %C = kron(eye(T),[-B eye(n)]) + kron([zeros(1,T); eye(T-1) zeros(T-1,1)],[zeros(n,m) -A]);
% C = [kron(eye(T),[-B eye(n)]) + kron([zeros(1,T); eye(T-1) zeros(T-1,1)],[zeros(n,m) -A]); zeros(ellef,m+(T-1)*(m+n)) Ef];
% 
% g = [r; repmat([q;r],T-1,1); qf];
% hc = [repmat(f,T,1);ff];
% hx = [-Fx; zeros((T-1)*ell + ellf,n)];
% % bx = [A; zeros((T-1)*n,n)];
% bx = [A; zeros(ellef+(T-1)*n,n)];
% b = [zeros(T*n,1); ef];
% 
% Ps = blkdiag(Fus,kron(eye(T-1),[Fxs Fus]));
% % check padding
% Ps(1,T*(n+m))=0;
% hcs = [repmat(fs,T,1)];
% hxs = [-Fxs; zeros((T-1)*ells,n)];
% 
% % extra bit for offset free tracking
% %bd = kron(ones(T,1),eye(n));
% bd = [kron(ones(T,1),eye(n)); kron(eye(2),[1 0 0 0; 0 1 0 0])];
% %bd = 0*bd;
% gt = kron(ones(T,1),[zeros(m,n); -2*Q]);
% 
% % modify soft constraints for dynamic limits, for avoidance
% hcp = zeros(length(hcs),4);
% % max x limits
% hcs(2:length(fs):end) = 0;
% hcp(2:length(fs):end,2) = 100;
% % min x limits
% hcs(1:length(fs):end) = 0;
% hcp(1:length(fs):end,1) = -100;
% % max y limits
% hcs(8:length(fs):end) = 0;
% hcp(8:length(fs):end,4) = 100;
% % min y limits
% hcs(7:length(fs):end) = 0;
% hcp(7:length(fs):end,3) = -100;

%% goof - try sparse
% seems to help quadprog anyway, provided interior point algorithm is used

%H = sparse(H);
%P = sparse(P);
%C = sparse(C);

%% MPC options
opts(1)=0; % 1 = assume diagonal Phi when solving
opts(2)=4; % Newton iterations
opts(3)=1; % 1 = use soft constraints


%% obstacle definitions

% goal point
goal = [1.8750    1.5000];

% obstacle boxes
% xmin xmax ymin ymax

% layout 1
obst = [-2.4950   -1.5000    1.3750    2.0000
    0.7000    1.2500   -1.2500    2.0000
   -1.1250   -0.5000   -1.1250    0.3500
   -1.8750   -0.6250   -2.0000   -1.5000
    0.6250    1.8750   -1.7500   -0.5000
    1.0000    1.2500   -3.0000   -2.1000];

% layout 2
obst = [-2.4950    1.5000    1.3750    2.0000
    0.7000    1.2500    1.2500    2.0000
   -1.1250   -0.5000   -1.1250    0.3500
   -1.8750   -0.6250   -2.0000   -1.5000
    0.6250    1.8750   -1.7500   -0.5000
    1.0000    1.2500   -3.0000   -2.1000];

% layout 3
% obst = [-2.4950    1.5000    1.3750    2.0000
%     0.7000    1.2500   -1.2500    2.0000
%    -1.1250   -0.5000   -1.1250    0.3500
%    -1.8750   -0.6250   -2.0000   -1.5000
%     0.6250    1.8750   -1.7500   -0.5000
%     1.0000    1.2500   -3.0000   -2.1000];

% layout 4
obst = [-2.4950    1.5000    1.3750    2.0000
    0.7000    1.2500   -1.2500    2.0000
   -1.1250   -0.5000   -1.1250    0.3500
   -1.8750    1.6250   -2.0000   -1.5000
    0.6250    1.8750   -1.7500   -0.5000
    1.0000    1.2500   -3.0000   -2.1000];


%% start position
homePos = [-2.5 -1];
%homePos = [-1 -2.5];

%% learning setup

% grid of nodes
[nodeX,nodeY] = meshgrid(-3:1:3,-4:1:3);
% reshape
nodeX = reshape(nodeX,numel(nodeX),1);
nodeY = reshape(nodeY,numel(nodeX),1);
% and pre-pend the goal
nodeX = [goal(1); nodeX];
nodeY = [goal(2); nodeY];

% initial cost proportional to distance
nodeV0 = (1/.4)*sqrt((nodeX-goal(1)).*(nodeX-goal(1))+(nodeY-goal(2)).*(nodeY-goal(2)));