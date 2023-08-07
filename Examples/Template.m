Ts = ?; % Sample time of the input and output
z = tf('z',Ts);

%% Name of the system

% Explicit system (TF ('z', 's') or NL (f, g))

% LTI
G = ?;
ss = ss(G);

% NL

f = @(t,x,u) ?; % Column vector
g = @(t,x,u) ?; % Column vector

% Initial condition
x0 = ?; % Column vector

%% System parameters
sys_params.sys_type = ?; %"LTI" or "NL"

% If LTI
sys_params.tf = G;
sys_params.ss = ss;

% If NL
sys_params.f = f;
sys_params.g = g;
sys_params.n_y = ?; % Number of outputs
sys_params.n_u = ?; % Number of inputs

sys_params.x0 = x0; 
sys_params.Ts = Ts; % Sample time of input and output

% Reference trajectory
sys_params.ref = @(t) ?;
sys_params.C_t = ?; % Tracking output

% C x C_c x Y + D <= 0
sys_params.C_c = ?; % Constraint output
sys_params.C = ?;
sys_params.D = ?;

% Standard deviation of the noise
sys_params.std_w = ?; % Input (scalar or matrix >= 0 )
sys_params.std_v = ?; % Output (scalar or matrix >= 0)

%% Recursive Least Squares (RLS) parameters 
rls_params.n_est = ?; %nhat

% Initialization of Theta_0/P_0
rls_params.Theta_0 = ?; % Scalar or column vector
rls_params.P_0 = ?; % Scalar or matrix

% Lambda parameters (specificy if you don't use VRF) 
rls_params.lambda = ?; % 1 >= Scalar > 0

% VRF parameters (DO NOT specify lambda)
rls_params.t_d = ?; % int > 0
rls_params.t_n = ?; % int > 0
rls_params.eta = ?; % scalar >= 0
rls_params.alpha = ?; % 1>= scalar >= 0 (DOESN'T WORK)

% Hypothesis on the estimated system
rls_params.properties = ["Strictly proper"];

%% Predictive Cost Adaptive Control (PCAC) parameters
pcac_params.nb_sample = ?; % Int > 0

% Input constraints
pcac_params.u_min = ?; % Scalar
pcac_params.u_max = ?; % Scalar
pcac_params.delta_u_min = ?; % Scalar
pcac_params.delta_u_max = ?; % Scalar

pcac_params.l = ?; % Horizon  % int > 0
pcac_params.Q_bar = ?; % Tracking error cost % Scalar >= 0 for 1-dim error, matrix >=0 for n-dim error
pcac_params.P_bar = ?; % Terminal tracking error cost  % Scalar >= 0 for 1-dim error, matrix >= 0 for n-dim error
pcac_params.R = ?; % Rate input cost  % Scalar >= 0 for 1-dim input, matrix >= 0 for n-dim input
pcac_params.S = ?; % Slack cost  % [] if no slack, Scalar >= 0 for 1-dim input, matrix >= 0 for n-dim constraints

%% DeePC Parameters

deepc_params.max_data = ?; %number of samples to create the Hankel Matrix
deepc_params.T_d = ?; %length of trials (DOESN'T WORK)
deepc_params.nb_try = ?; %number of trials (DOESN'T WORK)
deepc_params.T_ini = ?;
deepc_params.T_f = ?;
deepc_params.lambda_y_ini = ?;
deepc_params.lambda_u_ini = ?;
deepc_params.lambda_g = ?;
deepc_params.h = @(x) x'*x;
deepc_params.Proj_norm = 1; % If 1, h = ||(I-Pi)g||^2;

%%
params.sys_params = sys_params;
params.rls_params = rls_params;
params.pcac_params = pcac_params;
params.deepc_params = deepc_params;