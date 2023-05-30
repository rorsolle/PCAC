Ts = 0.05;
z = tf('z',Ts);

%% Deep Stall
% piecewise model
pw = aero.GtmPiecewise;

% system description longitudinal motion
% with states X = [V gamma qhat alpha]^T, inputs U = [eta T]^T
sys = eom.GtmLong(pw);

% find trim condition for eta = 0, T = 20 N
import aerootools.findtrim
x0 = findtrim(@sys.f, [35; 0; 0; 0.1], [0; 20]);

% Explicit system (TF ('z', 's') or NL (f, g))
f = @(t,x,u) sys.f(x,u);
g = @(t,x,u) x;

%% Recursive Least Squares (RLS) parameters 
rls_params.n_est = 3;

% Initialization of Theta_0/P_0
rls_params.Theta_0 = 0.1;
rls_params.P_0 = 10;

% Lambda parameters (specificy if you don't use VRF) 
rls_params.lambda = 0.95;

% VRF parameters (DO NOT specify lambda)
rls_params.t_d = 20; 
rls_params.t_n = 5;
rls_params.eta = 0.1;

% Hypothesis on the estimated system
rls_params.properties = ["Strictly proper"];

%% System
sys_params.sys_type = "NL";

% NL
sys_params.f = f;
sys_params.g = g;
sys_params.n_y = 4; % Number of outputs
sys_params.n_u = 2; % Number of inputs

sys_params.x0 = x0;
sys_params.Ts = Ts;

% Reference trajectory
sys_params.ref = @(t) x0.*ones(1,length(t));
sys_params.C_t = eye(4); % Tracking output

% C x C_c x Y + D <= 0
sys_params.C_c = [0,0,0,0]; % Constraint output
sys_params.C = 0;
sys_params.D = 0;

% Standard deviation of the noise
sys_params.std_w = 0; % Input
sys_params.std_v = 0; % Output

%% Predictive Cost Adaptive Control (PCAC) parameters
pcac_params.nb_sample = 100;

% Input constraints
pcac_params.u_min = [-deg2rad(60);0];
pcac_params.u_max = [deg2rad(20);inf];
pcac_params.delta_u_min = Ts*[-deg2rad(50);-50];
pcac_params.delta_u_max = Ts*[deg2rad(50);50];

pcac_params.l = 10; % Horizon
% Tracking error cost
pcac_params.Q_bar = 1*diag(1./[30-5, ...               % V
                               deg2rad(60-(-60)), ...  % Gamma
                               deg2rad(150-(-150)), ...% Qhat
                               deg2rad(75-(-10))]).^2; % Alpha


% Terminal tracking error cost
pcac_params.P_bar = 5*diag(1./[30-5, ...               % V
                               deg2rad(60-(-60)), ...  % Gamma
                               deg2rad(150-(-150)), ...% Qhat
                               deg2rad(75-(-10))]).^2; % Alpha

% Rate input cost
pcac_params.R = 1*diag(1./[deg2rad(20-(-60)), ... % Eta
                           100]).^2;              % T

%%
params.sys_params = sys_params;
params.rls_params = rls_params;
params.pcac_params = pcac_params;