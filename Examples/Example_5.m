params.save = 1;

omega=1;
omega_1=1;
omega_2=2;
Ts = 1/20*2*pi/omega;
z = tf('z',Ts);

%% Example 1

% Explicit system (TF ('z', 's') or NL (f, g))
A = [0,1,0,0;
     0,0,1,0;
     0,0,0,1;
     -omega^4,0,-2*omega^2,0];
B = [0;0;0;1]; C=[1,0,0,0];D=[0];
sys = c2d(ss(A,B,C,D),Ts);
sys = ss(c2d(tf([1],[1,0,2*omega^2,0,omega^4]),Ts)); 
sys = ss(c2d(tf([1],[1,0,(omega_1^2+omega_2^2),0,(omega_1*omega_2)^2]),Ts)); 
G = tf(sys);
% Initial condition
x0 = zeros(size(sys.A,1),1);
x0 = [1;0;0;0];

%% Recursive Least Squares (RLS) parameters 
rls_params.n_est = 4; %nhat

% Initialization of Theta_0/P_0
rls_params.Theta_0 = 0.1;
rls_params.P_0 = 1000;

% Lambda parameters (specificy if you don't use VRF) 
%rls_params.lambda = 1;

% VRF parameters (DO NOT specify lambda)
rls_params.t_d = 20; 
rls_params.t_n = 5;
rls_params.eta = 0.5;
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
sys_params.ref = @(t) [0].*ones(1,length(t));
%sys_params.ref = @(t) (t>=0).*(t<20) - (t>=20).*(t<40) + 3*(t>=40);%
sys_params.C_t = [1]; % Tracking output

% C x C_c x Y + D <= 0
sys_params.C_c = [0]; % Constraint output
sys_params.C = 0;
sys_params.D = 0;

% Standard deviation of the noise
sys_params.std_w = 0; % Input
sys_params.std_v = 0; % Output

%% Predictive Cost Adaptive Control (PCAC) parameters
pcac_params.nb_sample = 500;

% Input constraints
pcac_params.u_min = -0.1;
pcac_params.u_max = 0.1;
pcac_params.delta_u_min = -1;
pcac_params.delta_u_max = 1;

%pcac_params.l = ceil(1*2*pi/omega*1/Ts); % Horizon
pcac_params.l = 10; % Horizon
pcac_params.Q_bar = diag([1]); % Tracking error cost
pcac_params.P_bar = 1*diag([1]); % Terminal tracking error cost
pcac_params.R = 10^(-10); % Rate input cost
pcac_params.S = []; % Rate input cost

%% DeePC Parameters

deepc_params.T_d = 20;
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