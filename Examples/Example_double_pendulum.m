Ts = 0.1;
z = tf('z',Ts);

%% Double pendulum
m=1;l=1;g=9.81;
% https://en.wikipedia.org/wiki/Double_pendulum
t1p = @(t1,t2,p1,p2) 6/(m*l^2)*(2*p1-3*cos(t1-t2)*p2)/(16-9*cos(t1-t2)^2);
t2p = @(t1,t2,p1,p2) 6/(m*l^2)*(8*p2-3*cos(t1-t2)*p1)/(16-9*cos(t1-t2)^2);

EOM = @(t1,t2,p1,p2,u) [t1p(t1,t2,p1,p2);
                      t2p(t1,t2,p1,p2);
                      -1/2*m*l^2*(t1p(t1,t2,p1,p2)*t2p(t1,t2,p1,p2)*sin(t1-t2) + 3*g/l*sin(t1)) + m*g*u(1);
                      -1/2*m*l^2*(-t1p(t1,t2,p1,p2)*t2p(t1,t2,p1,p2)*sin(t1-t2) + g/l*sin(t2))  + m*g*u(2)];

f = @(t,x,u) EOM(x(1),x(2),x(3),x(4),u);
g = @(t,x,u) x;
x0 = zeros(4,1);
x0 = [pi/8;-pi/4;0;0]; % Initial condition

%% Recursive Least Squares (RLS) parameters 
rls_params.n_est = 3;

% Initialization of Theta_0/P_0
rls_params.Theta_0 = 0.01;
rls_params.P_0 = 1000;

% Lambda parameters (specificy if you don't use VRF) 
rls_params.lambda = 0.95;

% VRF parameters (DO NOT specify lambda)
rls_params.t_d = 20; 
rls_params.t_n = 5;
rls_params.eta = 0.1;

% Hypothesis on the estimated system
rls_params.properties = ["Strictly proper"];

%% System parameters
sys_params.sys_type = "NL";

% NL
sys_params.f = f;
sys_params.g = g;
sys_params.n_y = 4; % Number of outputs
sys_params.n_u = 2; % Number of inputs

sys_params.x0 = x0;
sys_params.Ts = Ts;

% Reference trajectory
sys_params.ref = @(t) [pi/4;pi/4].*ones(1,length(t));
%sys_params.ref = @(t) (t>=0).*(t<100) - (t>=100).*(t<200);%
%sys_params.ref = @(t) 0.2*sin(t/20);%
%sys_params.ref = @(t) 3*(sin(t/20)>0);%
sys_params.C_t = [1,0,0,0;
                  0,1,0,0]; % Tracking output

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
pcac_params.u_min = -[1;1]*2;
pcac_params.u_max = [1;1]*2;
pcac_params.delta_u_min = 2*[-4;-4]*Ts;
pcac_params.delta_u_max = 2*[4;4]*Ts;

pcac_params.l = 30; % Horizon
% Tracking error cost
pcac_params.Q_bar = 1*diag(1./[pi, ...  % Angle 1
                               pi]).^2; % Angle 2


% Terminal tracking error cost
pcac_params.P_bar = 1*diag(1./[pi, ...  % Angle 1
                               pi]).^2; % Angle 2

% Rate input cost
pcac_params.R = 1*diag(1./[1, ... % Torque 1
                           1]).^2;% Torque 2


%%
params.sys_params = sys_params;
params.rls_params = rls_params;
params.pcac_params = pcac_params;