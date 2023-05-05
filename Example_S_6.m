Ts = 1;
z = tf('z',Ts);

%% Example S6
G = 4*(z-1.2)/(z^2-z+1.2);

% RLS
rls_params.n_est = 2;
rls_params.t_d = 10;
rls_params.t_n = 2;
rls_params.lambda = 1;
rls_params.eta = 1;
rls_params.properties = ["Strictly proper"];

n_est = rls_params.n_est;

% System
sys_params.n_y = 1;
sys_params.n_u = 1;
n_y = sys_params.n_y;
n_u = sys_params.n_u;
sys_params.ref = @(t) ones(n_y,length(t));
sys_params.C_t = 1;
sys_params.C_c = 1;
sys_params.C = 0;
sys_params.D = 0;

% PCAC
pcac_params.u_min = -1;pcac_params.u_max = 1;
pcac_params.delta_u_min = -1;pcac_params.delta_u_max = 1;
pcac_params.l = 50;
pcac_params.Q_bar = 50*eye(pcac_params.l - 1);
pcac_params.P_bar = 50;
pcac_params.R = 10*eye(pcac_params.l);

% Initialization of Theta_0/P_0
Theta_0 = 0.01;
P_0 = 1000;

% Standard deviation of the noise
std_w = 0.1; % Input
std_v = 0.1; % Output