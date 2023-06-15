Ts = 0.1; % Sample time of the input and output
z = tf('z',Ts);

%% Name of the system

% Explicit system (TF ('z', 's') or NL (f, g))

% NL

alpha = 1.5; beta = 1; delta = 0.8;
f = @(t,x,u) [x(2);
              -alpha*x(1)^3+beta*x(1)-delta*x(2)+u]; % Column vector
g = @(t,x,u) x(1); % Column vector

% Initial condition
x0 = zeros(2,1); % Column vector
x0 = 200*rand(2,1)-100;% Column vector

%% Recursive Least Squares (RLS) parameters 
rls_params.n_est = 2; %nhat

% Initialization of Theta_0/P_0
rls_params.Theta_0 = [zeros(2*1*(1+1),1);ones(1*1,1)]; % Scalar or column vector
rls_params.P_0 = 1000; % Scalar or matrix

% Lambda parameters (specificy if you don't use VRF) 
rls_params.lambda = 0.9; % 1 >= Scalar > 0

% VRF parameters (DO NOT specify lambda)
rls_params.t_d = 40; % int > 0
rls_params.t_n = 20; % int > 0
rls_params.eta = 0.9; % scalar >= 0

% Hypothesis on the estimated system
rls_params.properties = ["Strictly proper"];

%% System parameters
sys_params.sys_type = "NL"; %"LTI" or "NL"

% If NL
sys_params.f = f;
sys_params.g = g;
sys_params.n_y = 1; % Number of outputs
sys_params.n_u = 1; % Number of inputs

sys_params.x0 = x0; 
sys_params.Ts = Ts; % Sample time of input and output

% Reference trajectory
sys_params.ref = @(t) zeros(size(t));
sys_params.C_t = 1; % Tracking output

% C x C_c x Y + D <= 0
sys_params.C_c = 0; % Constraint output
sys_params.C = 0;
sys_params.D = 0;

% Standard deviation of the noise
sys_params.std_w = 0; % Input (scalar or matrix >= 0 )
sys_params.std_v = 0; % Output (scalar or matrix >= 0)

%% Predictive Cost Adaptive Control (PCAC) parameters
pcac_params.nb_sample = 500; % Int > 0

% Input constraints
pcac_params.u_min = -10; % Scalar
pcac_params.u_max = 10; % Scalar
pcac_params.delta_u_min = -10; % Scalar
pcac_params.delta_u_max = 10; % Scalar

pcac_params.l = 20; % Horizon  % int > 0
pcac_params.Q_bar = 40; % Tracking error cost % Scalar >= 0 for 1-dim error, matrix >=0 for n-dim error
pcac_params.P_bar = 40; % Terminal tracking error cost  % Scalar >= 0 for 1-dim error, matrix >= 0 for n-dim error
pcac_params.R = 0.1; % Rate input cost  % Scalar >= 0 for 1-dim input, matrix >= 0 for n-dim input
pcac_params.S = [];%10*eye(size(sys_params.C,1));%*eye(pcac_params.l);

%%
params.sys_params = sys_params;
params.rls_params = rls_params;
params.pcac_params = pcac_params;