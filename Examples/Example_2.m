params.save = 0;

Ts = 1;
z = tf('z',Ts);

%% Example 2

% Explicit system (TF ('z', 's') or NL (f, g))
G = (z-1.3)/(z^2-1.4*z+0.3);
ss = ss(G);
% Initial condition
x0 = zeros(size(ss.A,1),1);

%% Recursive Least Squares (RLS) parameters 
rls_params.n_est = 2; %nhat

% Initialization of Theta_0/P_0
alpha = 1;
rls_params.Theta_0 = alpha*[zeros(1,2*rls_params.n_est),1]';
%rls_params.Theta_0 = 1*randn(2*rls_params.n_est+1,1);
rls_params.P_0 = 1000;

% Lambda parameters (specificy if you don't use VRF) 
rls_params.lambda = 1;

% VRF parameters (DO NOT specify lambda)
rls_params.t_d = 20; 
rls_params.t_n = 5;
rls_params.eta = 0.1;

% Hypothesis on the estimated system
rls_params.properties = ["Strictly proper"];

%% System parameters
sys_params.sys_type = "LTI"; %"LTI" or "NL"

% If LTI
sys_params.tf = {G};
sys_params.ss = {ss};
sys_params.switch = [];

% If NL
%sys_params.f = f;
%sys_params.g = g;

sys_params.x0 = x0; 
sys_params.Ts = Ts; % Sample time of input and output

% Reference trajectory
sys_params.ref = @(t) 1.*ones(1,length(t));
%sys_params.ref = @(t) (t>=0).*(t<20) - (t>=20).*(t<40) + 3*(t>=40);%
%sys_params.ref = @(t) sin(t/20);%
sys_params.C_t = 1; % Tracking output

% C x C_c x Y + D <= 0
sys_params.C_c = 0; % Constraint output
sys_params.C = 0;
sys_params.D = 0;

% Standard deviation of the noise
sys_params.std_w = 0; % Input
sys_params.std_v = 0; % Output

%% Predictive Cost Adaptive Control (PCAC) parameters
pcac_params.nb_sample = 60;

% Input constraints
pcac_params.u_min = -5;
pcac_params.u_max = 5;
pcac_params.delta_u_min = -5;
pcac_params.delta_u_max = 5;

pcac_params.l = 20;
pcac_params.Q_bar = 2; % Tracking error cost
pcac_params.P_bar = 5; % Terminal tracking error cost
pcac_params.R = 1; % Rate input cost
pcac_params.S = [];%*eye(size(sys_params.C,1)); % Rate input cost

%% DeePC Parameters

deepc_params.max_data = 1000;
deepc_params.T_d = 10;
deepc_params.nb_try = 5;
deepc_params.T_ini = 2;%rls_params.n_est;
deepc_params.T_f = pcac_params.l;
deepc_params.lambda_y_ini = 10^3;
deepc_params.lambda_u_ini = 10^3;
deepc_params.lambda_g = 0.01;
deepc_params.h = @(x) x'*x;
deepc_params.Proj_norm = 1;

%%
params.sys_params = sys_params;
params.rls_params = rls_params;
params.pcac_params = pcac_params;
params.deepc_params = deepc_params;