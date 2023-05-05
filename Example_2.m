Ts = 1;
z = tf('z',Ts);

%% Example 2
G = (z-1.3)/(z^2-1.4*z+0.3);

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
sys_params.ref = @(t) 1.*ones(n_y,length(t));
sys_params.C_t = 1;
sys_params.C_c = 1;
sys_params.C = 0;
sys_params.D = 0;

% PCAC
pcac_params.u_min = -50;
pcac_params.u_max = 50;
pcac_params.delta_u_min = -10;
pcac_params.delta_u_max = 10;
pcac_params.l = 20;
pcac_params.Q_bar = 2*eye(pcac_params.l - 1);
pcac_params.P_bar = 5;
pcac_params.R = 1*eye(pcac_params.l);

% Initialization of Theta_0/P_0
alpha = 1;
Theta_0 = alpha*[0,0,0,0,1]';
Theta_0 = 0.1;
P_0 = 1000;

% Standard deviation of the noise
std_w = 0; % Input
std_v = 0; % Output