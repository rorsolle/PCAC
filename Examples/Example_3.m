Ts = 1;
z = tf('z',Ts);
s = tf('s');

%% Example S5
G = (z-1.1)*(z-0.4)/((z-1.2)*(z^2+1.2*z+0.57));
%G = (s-0.1)*(s+0.6)/((s-0.2)*(s^2+2*0.1*(0.8*pi)*s+(0.8*pi)));

ss = ss(G);
x0 = zeros(size(ss.A,1),1);

%% RLS
rls_params.n_est = 3;

% Initialization of Theta_0/P_0
rls_params.Theta_0 = [zeros(1,2*rls_params.n_est),1]';
rls_params.P_0 = 1000;

rls_params.t_d = 10;
rls_params.t_n = 2;

rls_params.lambda = 1;
rls_params.eta = 1;
rls_params.properties = ["Strictly proper"];

%% System
sys_params.sys_type = "LTI";
sys_params.tf = G;
sys_params.ss = ss;
sys_params.x0 = x0;
sys_params.Ts = Ts;
%sys_params.ref = @(t) 1.*ones(1,length(t));
sys_params.ref = @(t) (t>=0).*(t<100) - (t>=100).*(t<200) + 2*(t>=200);%
sys_params.C_t = 1;
sys_params.C_c = 1;
sys_params.C = 0;
sys_params.D = 0;

% Standard deviation of the noise
sys_params.std_w = 0; % Input
sys_params.std_v = 0;%0.02; % Output

%% PCAC
pcac_params.nb_sample = 300;

pcac_params.u_min = -50;
pcac_params.u_max = 50;
pcac_params.delta_u_min = -10;
pcac_params.delta_u_max = 10;
pcac_params.l = 50;
pcac_params.Q_bar = 4;%*eye(pcac_params.l - 1);
pcac_params.P_bar = 4;
pcac_params.R = 1;%*eye(pcac_params.l);

%%
params.sys_params = sys_params;
params.rls_params = rls_params;
params.pcac_params = pcac_params;