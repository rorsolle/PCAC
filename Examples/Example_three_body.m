xi=1.215059*0.01;omega=2*pi/(655.728*3600);delta=384400;m=2000;
Ts = 0.01; % Sample time of the input and output
z = tf('z',Ts);

%% Name of the system

% Explicit system (TF ('z', 's') or NL (f, g))

% NL
r1 = @(x,y,z) sqrt((x+xi)^2+y^2+z^2);
r2 = @(x,y,z) sqrt((x-1+xi)^2+y^2+z^2);
f = @(t,x,u) [x(2);
              2*x(4)+x(1)-(1-xi)*(x(1)+xi)/r1(x(1),x(3),x(5))^3-xi*(x(1)-1+xi)/r2(x(1),x(3),x(5))^3+u(1)/(m*omega^2*delta);
              x(4);
              -2*x(2)+x(3)-(1-xi)*x(3)/r1(x(1),x(3),x(5))^3-xi*x(3)/r2(x(1),x(3),x(5))^3+u(2)/(m*omega^2*delta);
              x(6);
              -(1-xi)*x(5)/r1(x(1),x(3),x(5))^3-xi*x(5)/r2(x(1),x(3),x(5))^3+u(3)/(m*omega^2*delta)]; % Column vector
g = @(t,x,u) [x(1);x(3);x(5)]; % Column vector

% Initial condition
x0 = [1;0;0.1;0;0;0]; % Column vector

%% Recursive Least Squares (RLS) parameters 
rls_params.n_est = 2; %nhat

% Initialization of Theta_0/P_0
rls_params.Theta_0 = [zeros(rls_params.n_est*3*(3+3),1);ones(3*3,1)]; % Scalar or column vector
rls_params.P_0 = 1000; % Scalar or matrix

% Lambda parameters (specificy if you don't use VRF) 
%rls_params.lambda = ?; % 1 >= Scalar > 0

% VRF parameters (DO NOT specify lambda)
rls_params.t_d = 10; % int > 0
rls_params.t_n = 5; % int > 0
rls_params.eta = 0.1; % scalar >= 0

% Hypothesis on the estimated system
rls_params.properties = ["Strictly proper"];

%% System parameters
sys_params.sys_type = "NL"; %"LTI" or "NL"

% If NL
sys_params.f = f;
sys_params.g = g;
sys_params.n_y = 3; % Number of outputs
sys_params.n_u = 3; % Number of inputs

sys_params.x0 = x0; 
sys_params.Ts = Ts; % Sample time of input and output

% Reference trajectory
P = randn(3);
P = P/norm(P);
Tperiod=204.7;
sys_params.ref = @(t) P*[cos(t/Tperiod*2*pi);sin(t/Tperiod*2*pi);ones(size(t))]*0.1 + [1.75;0;0.1];
sys_params.C_t = eye(3); % Tracking output

% C x C_c x Y + D <= 0
sys_params.C_c = [0,0,0]; % Constraint output
sys_params.C = 0;
sys_params.D = 0;

% Standard deviation of the noise
sys_params.std_w = 0; % Input (scalar or matrix >= 0 )
sys_params.std_v = 0; % Output (scalar or matrix >= 0)

%% Predictive Cost Adaptive Control (PCAC) parameters
pcac_params.nb_sample = 6*205; % Int > 0

% Input constraints
pcac_params.u_min = -1*[1;1;1]; % Scalar
pcac_params.u_max = 1*[1;1;1]; % Scalar
pcac_params.delta_u_min = -0.5*1*[1;1;1]; % Scalar
pcac_params.delta_u_max = 0.5*1*[1;1;1]; % Scalar

pcac_params.l = 20; % Horizon  % int > 0
pcac_params.Q_bar = 40*eye(3); % Tracking error cost % Scalar >= 0 for 1-dim error, matrix >=0 for n-dim error
pcac_params.P_bar = 40*eye(3); % Terminal tracking error cost  % Scalar >= 0 for 1-dim error, matrix >= 0 for n-dim error
pcac_params.R = eye(3); % Rate input cost  % Scalar >= 0 for 1-dim input, matrix >= 0 for n-dim input
pcac_params.S = [];%10*eye(size(sys_params.C,1));%*eye(pcac_params.l);

%%
params.sys_params = sys_params;
params.rls_params = rls_params;
params.pcac_params = pcac_params;