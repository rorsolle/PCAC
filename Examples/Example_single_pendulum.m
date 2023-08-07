params.save = 0;
Ts = 0.1;
z = tf('z',Ts);

%% Single pendulum

f = @(t,x,u) [x(2);-9.81/1*sin(x(1))+9.81*u];
g = @(t,x,u) x;

x0 = [pi/2;-0.01]; % Initial condition

%% System parameters
sys_params.sys_type = "NL";

% NL
sys_params.f   = f;
sys_params.g   = g;
sys_params.n_y = 2; % Number of outputs
sys_params.n_u = 1; % Number of inputs

sys_params.x0 = x0;
sys_params.Ts = Ts;

% Reference trajectory
%sys_params.ref = @(t) [pi].*ones(1,length(t));
%sys_params.ref = @(t) (t>=0).*(t<100) - (t>=100).*(t<200);%
%sys_params.ref = @(t) pi*(sin(t/30)>0);%
sys_params.ref = @(t) [pi].*ones(1,length(t));
sys_params.C_t = [1,0]; % Tracking output

% C x C_c x Y + D <= 0
c = 10;
sys_params.C_c = [0,0];%[1,0,0;0,1,0];
sys_params.C = 0;%0*[1,c;
                %-1,c];%[1;-1];
sys_params.D = 0;%[-2*c;-2*c];

% Standard deviation of the noise
sys_params.std_w = 0; % Input
sys_params.std_v = 0; % Output

%% Recursive Least Squares (RLS) parameters 
rls_params.n_est = 3;

% Initialization of Theta_0/P_0
rls_params.Theta_0 = 0.1;
%rls_params.Theta_0 = linearize_guess(x0,sys_params,rls_params); % Scalar or column vector

rls_params.P_0 = 1000;

% Lambda parameters (specificy if you don't use VRF) 
%rls_params.lambda = 0.9;

% VRF parameters (DO NOT specify lambda)
rls_params.t_d = 50; 
rls_params.t_n = 5;
rls_params.eta = 0.1;
rls_params.alpha = 0.1;

% Hypothesis on the estimated system
rls_params.properties = ["Strictly proper"];

%% Predictive Cost Adaptive Control (PCAC) parameters
pcac_params.nb_sample = 300;

% Input constraints
pcac_params.u_min = -1;
pcac_params.u_max = 1;
pcac_params.delta_u_min = -1;
pcac_params.delta_u_max = 1;

pcac_params.l = 10; % Horizon

% Tracking error cost
pcac_params.Q_bar = 10*diag([1]);%1/1^2*eye(size(sys_params.C_t,1));%*eye(pcac_params.l - 1);

% Terminal tracking error cost
pcac_params.P_bar = 10*diag([1]);%100/1^2*eye(size(sys_params.C_t,1));

% Rate input cost
pcac_params.R = 1/1^2*eye(sys_params.n_u);%*eye(pcac_params.l);
pcac_params.S = [];%*eye(size(sys_params.C,1));%*eye(pcac_params.l);


%% DeePC Parameters

deepc_params.max_data = 40;
deepc_params.T_d = 10;
deepc_params.nb_try = 5;
deepc_params.T_ini = 3;%rls_params.n_est;
deepc_params.T_f = 20;
deepc_params.lambda_y_ini = 10^3;
deepc_params.lambda_u_ini = 10^3;
deepc_params.lambda_g = 10;
deepc_params.h = @(x) x'*x;
deepc_params.Proj_norm = 0;

%%
params.sys_params = sys_params;
params.rls_params = rls_params;
params.pcac_params = pcac_params;
params.deepc_params = deepc_params;