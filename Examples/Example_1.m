params.save = 0;

Ts = 1;
z = tf('z',Ts);

%% Example 1

% Explicit system (TF ('z', 's') or NL (f, g))
G = (z-0.4)/(z^2+0.5*z-0.1);
sys = ss(G);
% Initial condition
x0 = zeros(size(sys.A,1),1);

%% Recursive Least Squares (RLS) parameters 
rls_params.n_est = 2; %nhat

% Initialization of Theta_0/P_0
rls_params.Theta_0 = 0.01;
rls_params.P_0 = 1;

% Lambda parameters (specificy if you don't use VRF) 
rls_params.lambda = 1;

% VRF parameters (DO NOT specify lambda)
rls_params.t_d = 15; 
rls_params.t_n = 5;
rls_params.eta = 0.9;
rls_params.alpha = 0.2;

% Hypothesis on the estimated system
rls_params.properties = ["Strictly proper"];

%% System parameters
sys_params.sys_type = "LTI"; %"LTI" or "NL"

% If LTI
sys_params.tf = {G};
sys_params.ss = {sys};
sys_params.switch = [];

sys_params.x0 = x0; 
sys_params.Ts = Ts; % Sample time of input and output

% Reference trajectory
sys_params.ref = @(t) 1.*ones(1,length(t));
%sys_params.ref = @(t) (t>=0).*(t<20) - (t>=20).*(t<40) + 3*(t>=40);%
%sys_params.ref = @(t) sin(2*pi*t.^2/100);%
sys_params.C_t = 1; % Tracking output

% C x C_c x Y + D <= 0
sys_params.C_c = 1; % Constraint output
sys_params.C = 0;
sys_params.D = 0;

% Standard deviation of the noise
sys_params.std_w = 0; % Input
sys_params.std_v = 0; % Output

%% Predictive Cost Adaptive Control (PCAC) parameters
pcac_params.nb_sample = 100;

% Input constraints
pcac_params.u_min = -10;
pcac_params.u_max = 10;
pcac_params.delta_u_min = -10;
pcac_params.delta_u_max = 10;

pcac_params.l = 5; % Horizon
pcac_params.Q_bar = 2; % Tracking error cost
pcac_params.P_bar = 5; % Terminal tracking error cost
pcac_params.R = 0.0001;%1/2^2; % Rate input cost
pcac_params.S = []; % Rate input cost

%% DeePC Parameters

deepc_params.max_data = 1000;
deepc_params.T_d = 10;
deepc_params.nb_try = 5;
deepc_params.T_ini = 2;%rls_params.n_est;
deepc_params.T_f = pcac_params.l;
deepc_params.lambda_y_ini = 10;
deepc_params.lambda_u_ini = 10;
deepc_params.lambda_g = 0.01;
deepc_params.h = @(x) x'*x;
deepc_params.Proj_norm = 0;

%% ODeePC Parameters

odeepc_params.N_l = 10;
odeepc_params.kappa = 30;
odeepc_params.alpha = 0.1;
odeepc_params.eps_g = 0.1;
odeepc_params.eps_v = 0.1;


%%
params.sys_params = sys_params;
params.rls_params = rls_params;
params.pcac_params = pcac_params;
params.deepc_params = deepc_params;
params.odeepc_params = odeepc_params;