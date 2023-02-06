run highFidelityTiltingQuadrotorData.m


%% Simulation parameters
if exist('settings','file')==2
    load('MATMPC/data/settings');
    if( strcmp(settings.model,'highFidelityTiltingQuadrotor')==0 )
        error('wrong model selected! Recompile the model')
    end
else
    error('No setting data is detected! Recompile the model');
end

Ts  = T_s;    % Sampling time for the dynamics

Ts_st = settings.Ts_st;  % Shooting interval
nx = settings.nx;        % No. of differential states
nu = settings.nu;        % No. of controls
nz = settings.nz;        % No. of algebraic states
ny = settings.ny;        % No. of outputs (references)    
nyN= settings.nyN;       % No. of outputs at terminal stage 
np = settings.np;        % No. of parameters (on-line data)
nc = settings.nc;        % No. of constraints
ncN = settings.ncN;      % No. of constraints at terminal stage
nbu = settings.nbu;      % No. of control bounds
nbx = settings.nbx;      % No. of state bounds
nbu_idx = settings.nbu_idx; % Index of control bounds
nbx_idx = settings.nbx_idx; % Index of state bounds

%% add more to Settings

N  = 30;             % No. of shooting points
settings.N = N;

N2 = 1;
settings.N2 = N2;    % No. of horizon length after partial condensing (N2=1 means full condensing)

r = 10;
settings.r = r;      % No. of input blocks (go to InitMemory.m, line 441 to configure)

%% options
opt.hessian         = 'Gauss_Newton';  % 'Gauss_Newton', 'Generalized_Gauss_Newton'
opt.integrator      = 'ERK4'; % 'ERK4','IRK3, 'IRK3-DAE'
opt.condensing      = 'no';  %'default_full','no','blasfeo_full(require blasfeo installed)','partial_condensing'
opt.qpsolver        = 'hpipm_sparse';
opt.hotstart        = 'no'; %'yes','no' (only for qpoases)
opt.shifting        = 'yes'; % 'yes','no'
opt.ref_type        = 2; % 0-time invariant, 1-time varying(no preview), 2-time varying (preview)
opt.nonuniform_grid = 0; % currently not supported
opt.RTI             = 'yes';

%% available qpsolver
%'qpoases' (for full condensing)
%'qpoases_mb' (for full condensing+moving block, please use ERK4 as the integrator)
%'hpipm_sparse' (run mex_core/compile_hpipm.m first; set opt.condensing='no')
%'hpipm_pcond' (run mex_core/compile_hpipm.m first; set opt.condensing='no')

%% reference
load benchmark_trajectories/b04.mat

time = tt';
ref.time = time;
ref.signals.dimensions = [ny, N];
ref.signals.values = zeros([ny, N, length(time)]);
for k=1:length(time)-N
    ref.signals.values(:,:,k) = reference(k:k+N-1, 1:ny)';
end
refN.time = time;
refN.signals.dimensions = nyN;
refN.signals.values = zeros([length(time), nyN]);
refN.signals.values(1:length(time)-N,:) = reference(N+1:end, 1:nyN);
q_des.time = time;
q_des.signals.dimensions = [4, N+1];
q_des.signals.values = zeros([4, N+1, length(time)]);
for k=1:length(time)-N-1
    q_des.signals.values(:,:,k) = des_q(:, k:k+N);
end

 
%% Initialization

x_MPC_0 = [
    zeros(3,1)
    [1;0;0;0]
    zeros(3,1)
    zeros(3,1)
    zeros(4,1)
    0.33*ones(4,1)
    ];    
u_MPC_0 = [
    zeros(4,1)
    zeros(4,1)
    ]; 
z0 = zeros(nz,1);
para0 = [1;0;0;0];  

Qp = 50*[1 1 1];                 % position weight
Qeq = [1 1 1];                        % quaternion weight
Qv = 0.5*[1 1 1];               % linear speed weight
Qomega = 10*[1 1 1];            % angular speed weight
Qalpha = 1e-2*[1 1 1 1];        % alpha angle weight
Qw_bar2 = [1 1 1 1]*1e-3;       % propellers spinning rate square weight
Qup = [1 1 1 1]*1e-1;           % propellers spinning acceleration square weight
Qdotalpha = 1e-3*[1 1 1 1];     % alpha dot weight
W=repmat([Qp Qeq Qv Qomega Qalpha Qw_bar2 Qup Qdotalpha]',1, N);
WN=[Qp Qeq Qv Qomega Qalpha Qw_bar2]';

% upper and lower bounds for states (=nbx)
lb_x = [
    -pi/3 .* ones(4,1)  % alpha min: -60 deg
    0.01.*ones(4,1)     % w_bar2 min: 0.01 of max
    ];
ub_x = [
    pi/3 .* ones(4,1)   % alpha max: 60 deg
    ones(4,1)           % w_bar2 max: 100% of max
    ];

% upper and lower bounds for controls (=nbu)           
lb_u = [
    -10*ones(4,1)       % w_bar2_dot_min: -10 s^{-1}
    -6*ones(4,1)        % alpha_dot_min: -6 rad/s
    ]; % MIN SPEED AT 1%
ub_u = [
    10*ones(4,1)        % w_bar2_dot_min: 10 s^{-1}
    6*ones(4,1)         % alpha_dot_max: 6 rad/s
    ];
                       
% upper and lower bounds for general constraints (=nc)
lb_g = [];
ub_g = [];            
lb_gN = [];
ub_gN = [];

%%
lb = repmat(lb_g,N,1);
ub = repmat(ub_g,N,1);
lb = [lb;lb_gN];
ub = [ub;ub_gN];
if isempty(lb)
    lb=0;
    ub=0;
end
        
lbu = -inf(nu,1);
ubu = inf(nu,1);
for i=1:nbu
    lbu(nbu_idx(i)) = lb_u(i);
    ubu(nbu_idx(i)) = ub_u(i);
end
        
lbu = repmat(lbu,1,N);
ubu = repmat(ubu,1,N);

lbx = repmat(lb_x,1,N+1);
ubx = repmat(ub_x,1,N+1);
if isempty(lbx)
    lbx=0;
    ubx=0;
end

x = repmat(x_MPC_0,1,N+1);   
u = repmat(u_MPC_0,1,N);  
z = repmat(z0,1,N);
para = repmat(para0,1,N+1);  

if isempty(z)
    z0=0;
    z=0;
end
