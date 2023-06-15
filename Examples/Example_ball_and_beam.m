Ts = 0.1; % Sample time of the input and output
z = tf('z',Ts);

%% Name of the system

% Explicit system (TF ('z', 's') or NL (f, g))

% NL
g=9.81;m=0.1;J=0.3;
f = @(t,x,u) [x(2);
              x(1).*x(4).^2-g.*sin(x(3));
              x(4);
              -m*x(1)./(m*x(1).^2+J).*(2*x(2).*x(4)+g*cos(x(3))) + u/(m*x(1).^2+J)]; % Column vector
G = @(t,x,u) [x(1);x(3)]; % Column vector

% Initial condition
x0 = [0.05;0;0;0]; % Column vector

%% System parameters
sys_params.sys_type = "NL"; %"LTI" or "NL"

% If NL
sys_params.f = f;
sys_params.g = G;
sys_params.n_y = 2; % Number of outputs
sys_params.n_u = 1; % Number of inputs

sys_params.x0 = x0; 
sys_params.Ts = Ts; % Sample time of input and output

% Reference trajectory
sys_params.ref = @(t) zeros(1,length(t));
sys_params.C_t = [1,0]; % Tracking output

% C x C_c x Y + D <= 0
sys_params.C_c = eye(2); % Constraint output
sys_params.C = [1,0;
                -1,0;
                0,1;
                0,-1];
sys_params.D = [-1;
                -1;
                -pi/4;
                -pi/4];

% Standard deviation of the noise
sys_params.std_w = 0; % Input (scalar or matrix >= 0 )
sys_params.std_v = 0; % Output (scalar or matrix >= 0)

%% Recursive Least Squares (RLS) parameters 
rls_params.n_est = 2; %nhat

% Initialization of Theta_0/P_0
rls_params.Theta_0 = [-2,0.43,1,-0.38,0,0.09,0,-1.8,0,0,0,0.77,0.03,0.01]'; % Scalar or column vector
rls_params.Theta_0 = linearize_guess(x0,sys_params,rls_params); % Scalar or column vector

rls_params.P_0 = 1000; % Scalar or matrix

% Lambda parameters (specificy if you don't use VRF) 
%rls_params.lambda = 0.9; % 1 >= Scalar > 0

% VRF parameters (DO NOT specify lambda)
rls_params.t_d = 40; % int > 0
rls_params.t_n = 20; % int > 0
rls_params.eta = 0.9; % scalar >= 0

% Hypothesis on the estimated system
rls_params.properties = ["Strictly proper"];

%% Predictive Cost Adaptive Control (PCAC) parameters
pcac_params.nb_sample = 200; % Int > 0

% Input constraints
pcac_params.u_min = -1; % Scalar
pcac_params.u_max = 1; % Scalar
pcac_params.delta_u_min = -0.5; % Scalar
pcac_params.delta_u_max = 0.5; % Scalar

pcac_params.l = 20; % Horizon  % int > 0
pcac_params.Q_bar = 40; % Tracking error cost % Scalar >= 0 for 1-dim error, matrix >=0 for n-dim error
pcac_params.P_bar = 40; % Terminal tracking error cost  % Scalar >= 0 for 1-dim error, matrix >= 0 for n-dim error
pcac_params.R = 1; % Rate input cost  % Scalar >= 0 for 1-dim input, matrix >= 0 for n-dim input
pcac_params.S = [];%5*1000; % Rate input cost  % Scalar >= 0 for 1-dim input, matrix >= 0 for n-dim input

%%
params.sys_params = sys_params;
params.rls_params = rls_params;
params.pcac_params = pcac_params;