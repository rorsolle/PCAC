Ts = 0.1;
z = tf('z',Ts);

%% Single pendulum

f = @(t,x,u) [x(2);-9.81/1*sin(x(1))+9.81*u];
g = @(t,x,u) x;

x0 = zeros(2,1);
x0 = [0;0]; % Initial condition

%% Recursive Least Squares (RLS) parameters 
rls_params.n_est = 4;

% Initialization of Theta_0/P_0
rls_params.Theta_0 = 0.1;
rls_params.P_0 = 100;

% Lambda parameters (specificy if you don't use VRF) 
rls_params.lambda = 0.85;

% VRF parameters (DO NOT specify lambda)
rls_params.t_d = 20; 
rls_params.t_n = 5;
rls_params.eta = 0.1;

% Hypothesis on the estimated system
rls_params.properties = ["Strictly proper"];

%% System parameters
sys_params.sys_type = "NL";

% NL
sys_params.f = f;
sys_params.g = g;
sys_params.n_y = 2; % Number of outputs
sys_params.n_u = 1; % Number of inputs

sys_params.x0 = x0;
sys_params.Ts = Ts;

% Reference trajectory
%sys_params.ref = @(t) [pi].*ones(1,length(t));
%sys_params.ref = @(t) (t>=0).*(t<100) - (t>=100).*(t<200);%
%sys_params.ref = @(t) 0.2*sin(t/20);%
sys_params.ref = @(t) pi*(t>0);
sys_params.C_t = [1,0]; % Tracking output

% C x C_c x Y + D <= 0
sys_params.C_c = [0,0];
sys_params.C = 0;
sys_params.D = 0;

% Standard deviation of the noise
sys_params.std_w = 0; % Input
sys_params.std_v = 0; % Output

%% Predictive Cost Adaptive Control (PCAC) parameters
pcac_params.nb_sample = 200;

% Input constraints
pcac_params.u_min = -0.5;
pcac_params.u_max = 0.5;
pcac_params.delta_u_min = -4*Ts;
pcac_params.delta_u_max = 4*Ts;

pcac_params.l = 20; % Horizon

% Tracking error cost
pcac_params.Q_bar = 0/40*eye(size(sys_params.C_t,1));%*eye(pcac_params.l - 1);

% Terminal tracking error cost
pcac_params.P_bar = 5*eye(size(sys_params.C_t,1));

% Rate input cost
pcac_params.R = 0*eye(sys_params.n_u);%*eye(pcac_params.l);

%%
params.sys_params = sys_params;
params.rls_params = rls_params;
params.pcac_params = pcac_params;