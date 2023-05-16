Ts = 1;
z = tf('z',Ts);

%% Example S5
G = (z-0.4)/(z^2+0.5*z-0.1);

%% RLS
rls_params.n_est = 2;

% Initialization of Theta_0/P_0
rls_params.Theta_0 = 0.01;
rls_params.P_0 = 1000;

rls_params.t_d = 10;
rls_params.t_n = 2;

rls_params.lambda = 1;
rls_params.eta = 1;
rls_params.properties = ["Strictly proper"];

%% System
sys_params.tf = G;
sys_params.ref = @(t) 1.*ones(1,length(t));
sys_params.ref = @(t) (t>=0).*(t<20) - (t>=20).*(t<40) + 3*(t>=40);%
%sys_params.ref = @(t) sin(t/20);%
sys_params.C_t = 1;
sys_params.C_c = 1;
sys_params.C = 0;
sys_params.D = 0;

% Standard deviation of the noise
sys_params.std_w = 0; % Input
sys_params.std_v = 0; % Output

%% PCAC
pcac_params.nb_sample = 60;

pcac_params.u_min = -10;
pcac_params.u_max = 10;
pcac_params.delta_u_min = -10;
pcac_params.delta_u_max = 10;
pcac_params.l = 5;
pcac_params.Q_bar = 2;%*eye(pcac_params.l - 1);
pcac_params.P_bar = 5;
pcac_params.R = 1;%*eye(pcac_params.l);

%%
params.sys_params = sys_params;
params.rls_params = rls_params;
params.pcac_params = pcac_params;

params = format_parameters(params);