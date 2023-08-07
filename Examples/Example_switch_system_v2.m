params.save = 0;

Ts = 1;
z = tf('z',Ts);

%% Example 1

% Explicit system (TF ('z', 's') or NL (f, g))
G1 = (z-0.4)/(z^2+0.5*z-0.1);
ss1 = ss(d2c(G1));
ss2 = ss(ss1.A*1.1,ss1.B*1.1,ss1.C,ss1.D,ss1);
f = @(t,x,u) (t<100).*(ss1.A*x + ss1.B*u) + (t>=100).*(ss2.A*x + ss2.B*u);
g = @(t,x,u) (t<100).*(ss1.C*x + ss1.D*u) + (t>=100).*(ss2.C*x + ss2.D*u);

% Initial condition
x0 = zeros(size(ss1.A,1),1);

%% System parameters
sys_params.sys_type = "NL";

% NL
sys_params.f = f;
sys_params.g = g;
sys_params.n_y = 1; % Number of outputs
sys_params.n_u = 1; % Number of inputs

sys_params.x0 = x0;
sys_params.Ts = Ts;

% Reference trajectory
sys_params.ref = @(t) [1].*ones(1,length(t));
sys_params.C_t = 1; % Tracking output

% C x C_c x Y + D <= 0
sys_params.C_c = 0;
sys_params.C = 0;
sys_params.D = 0;

% Standard deviation of the noise
sys_params.std_w = 0; % Input
sys_params.std_v = 0; % Output

%% Recursive Least Squares (RLS) parameters 
rls_params.n_est = 2; %nhat

% Initialization of Theta_0/P_0
rls_params.Theta_0 = 0.01;
rls_params.P_0 = 1000;

% Lambda parameters (specificy if you don't use VRF) 
%rls_params.lambda = 1;

% VRF parameters (DO NOT specify lambda)
rls_params.t_d = 10; 
rls_params.t_n = 5;
rls_params.eta = 0.1;

% Hypothesis on the estimated system
rls_params.properties = ["Strictly proper"];

%% Predictive Cost Adaptive Control (PCAC) parameters
pcac_params.nb_sample = 200;

% Input constraints
pcac_params.u_min = -10;
pcac_params.u_max = 10;
pcac_params.delta_u_min = -10;
pcac_params.delta_u_max = 10;

pcac_params.l = 5; % Horizon
pcac_params.Q_bar = 2; % Tracking error cost
pcac_params.P_bar = 5; % Terminal tracking error cost
pcac_params.R = 1;%1/2^2; % Rate input cost
pcac_params.S = []; % Rate input cost

%% DeePC Parameters

deepc_params.T_d = 10;
deepc_params.nb_try = 5;
deepc_params.T_ini = rls_params.n_est;
deepc_params.T_f = pcac_params.l;
deepc_params.lambda_y_ini = 1000;
deepc_params.lambda_u_ini = 1000;
deepc_params.lambda_g = 0;
deepc_params.h = @(x) x'*x;
deepc_params.Proj_norm = 0;

%%
params.sys_params = sys_params;
params.rls_params = rls_params;
params.pcac_params = pcac_params;
params.deepc_params = deepc_params;