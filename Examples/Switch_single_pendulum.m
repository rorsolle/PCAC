params.save = 0;
Ts = 0.1;
z = tf('z',Ts);

%% Single pendulum

omega=sqrt(9.81);
A_1 = [0,1;-omega^2,0];
A_2 = [0,1;omega^2,0];
B = 9.81*[0;1]; C=[1,0];D=[0];

f = @(t,x,u) (abs(mod(x(1)+pi,2*pi)-pi)<2*pi/3)*(A_1*x+B*u) +...
             (abs(mod(x(1)+pi,2*pi)-pi)>=2*pi/3)*(A_2*(x-[pi;0])+B*u);
%f = @(t,x,u) A_1*x+B*u;
g = @(t,x,u) x;%[sin(x(1)) ; 1-cos(x(1)) ; x(2)];

x0 = [2*pi/4;0]; % Initial condition

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
%sys_params.ref = @(t) pi*(sin(t/30)>0);%
sys_params.ref = @(t) [pi].*ones(1,length(t));
sys_params.C_t = [1,0]; % Tracking output

% C x C_c x Y + D <= 0
sys_params.C_c = [0,0];
sys_params.C = 0;
sys_params.D = 0;

% Standard deviation of the noise
sys_params.std_w = 0; % Input
sys_params.std_v = 0; % Output

%% Recursive Least Squares (RLS) parameters 
rls_params.n_est = 2;

% Initialization of Theta_0/P_0
rls_params.Theta_0 = 0.1;
%rls_params.Theta_0 = linearize_guess(x0,sys_params,rls_params); % Scalar or column vector

rls_params.P_0 = 1000;

% Lambda parameters (specificy if you don't use VRF) 
%rls_params.lambda = 0.9;

% VRF parameters (DO NOT specify lambda)
rls_params.t_d = 30; 
rls_params.t_n = 10;
rls_params.eta = 0.5;
rls_params.alpha = 0.1;

% Hypothesis on the estimated system
rls_params.properties = ["Strictly proper"];

%% Predictive Cost Adaptive Control (PCAC) parameters
pcac_params.nb_sample = 200;

% Input constraints
pcac_params.u_min = -0.5;
pcac_params.u_max = 0.5;
pcac_params.delta_u_min = -4;
pcac_params.delta_u_max = 4;

pcac_params.l = 20; % Horizon

% Tracking error cost
pcac_params.Q_bar = 1/1^2*eye(size(sys_params.C_t,1));%*eye(pcac_params.l - 1);

% Terminal tracking error cost
pcac_params.P_bar = 1/1^2*eye(size(sys_params.C_t,1));

% Rate input cost
pcac_params.R = 0.00000000000000001/1^2*eye(sys_params.n_u);%*eye(pcac_params.l);
pcac_params.S = [];%*eye(size(sys_params.C,1));%*eye(pcac_params.l);


%% DeePC Parameters

deepc_params.T_d = 25;
deepc_params.nb_try = 15;
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