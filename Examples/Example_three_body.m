%https://ssd.jpl.nasa.gov/tools/periodic_orbits.html#/intro
 params.save = 0;

ratio = 1.215058560962404E-2;
LU = 389703;
TU = 382981;
xi=ratio/(1+ratio); %m1/(m1+m2)
omega=2*pi/(655.728*3600); % rotation rate for celestial bodies (rad/s)
%omega=2*pi/(TU); % rotation rate for celestial bodies (rad/s)
tc = sqrt(LU^3/xi);
%delta=384400; % distance between the two celestial bodies (km)
delta=LU; % distance between the two celestial bodies (km)
m=2000; % weight of the spacecraft (kg)

Ts = 0.01; % Sample time of the input and output
z = tf('z',Ts);

%% Name of the system

% Explicit system (TF ('z', 's') or NL (f, g))

% NL
r1 = @(x,y,z) sqrt((x+xi)^2+y^2+z^2);
r2 = @(x,y,z) sqrt((x-1+xi)^2+y^2+z^2);
f = @(t,x,u) [x(2);
              2*x(4)+x(1)-(1-xi)*(x(1)+xi)/r1(x(1),x(3),x(5))^3-xi*(x(1)-1+xi)/r2(x(1),x(3),x(5))^3+u(1)/(m*delta*omega^2);
              x(4);
              -2*x(2)+x(3)-(1-xi)*x(3)/r1(x(1),x(3),x(5))^3-xi*x(3)/r2(x(1),x(3),x(5))^3+u(2)/(m*delta*omega^2);
              x(6);
              -(1-xi)*x(5)/r1(x(1),x(3),x(5))^3-xi*x(5)/r2(x(1),x(3),x(5))^3+u(3)/(m*delta*omega^2);
              ]; % Column vector

g = @(t,x,u) [x(1);x(3);x(5)]; % Column vector

% Initial condition for halo orbit

% Orbit v1
pos_0 = [
-1.6967233092486720E+0;
0;%3.1756007279014768E-23;
1.0136396075463166E-2
         ]*LU/delta;

v_0 = [
0;%-5.8932494929994690E-13;
1.2795419587406101E+0;
0;%-1.3980383346242373E-14	
        ]*(LU/delta)*1/(TU/(2*pi/omega));


% Orbit v2
% pos_0 = [
% 4.8784941344943100E-1;
% -1.4781979515535899E+0;
% -2.1667662873886964E-24
% ];
% 
% v_0 = [
% -1.0314326086342718E+0;
% -7.7897933109135520E-1;
% -3.0299271073821780E-25
% ];

% Initial condition spacecraft
pos_0 = pos_0;% + 0.2*(2*rand(3,1)-1);%[0.7258;-0.1548;0.9545];
v_0 = v_0;% + 0.2*(2*rand(3,1)-1);%[-0.9240;-1.4621;0.0663];
x0 = [
    pos_0(1); v_0(1);
    pos_0(2); v_0(2);
    pos_0(3); v_0(3);
    ];

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
% P = randn(3);
% P = P/norm(P);
% Tperiod=204.7;
% sys_params.ref = @(t) P*[cos(t/Tperiod*2*pi);sin(t/Tperiod*2*pi);ones(size(t))]*0.1 + [1.75;0;0.1];

orbit = open('halo_orbit.mat'); T=624; % Orbit 1
%orbit = open('halo_orbit_2.mat'); T=629; % Orbit 2
orbit = orbit.orbit;
orbit = kron(ones(1,30),orbit(:,1:T));
sys_params.ref = @(t) orbit(:,t);
sys_params.C_t = eye(3); % Tracking output

% C x C_c x Y + D <= 0
sys_params.C_c = [0,0,0]; % Constraint output
sys_params.C = 0;
sys_params.D = 0;

% Standard deviation of the noise
sys_params.std_w = 0; % Input (scalar or matrix >= 0 )
sys_params.std_v = 0; % Output (scalar or matrix >= 0)

%% Recursive Least Squares (RLS) parameters 
rls_params.n_est = 2; %nhat

% Initialization of Theta_0/P_0
rls_params.Theta_0 = [zeros(rls_params.n_est*sys_params.n_y*(sys_params.n_y+sys_params.n_u),1);ones(sys_params.n_y*sys_params.n_u,1)]; % Scalar or column vector
%rls_params.Theta_0 = linearize_guess(x0,sys_params,rls_params); % Scalar or column vector
rls_params.P_0 = 10^2; % Scalar or matrix

% Lambda parameters (specificy if you don't use VRF) 
%rls_params.lambda = 1; % 1 >= Scalar > 0

% VRF parameters (DO NOT specify lambda)
rls_params.t_d = 20; % int > 0
rls_params.t_n = 5; % int > 0
rls_params.eta = 0.1; % scalar >= 0
rls_params.alpha = 0.1; % scalar >= 0

% Hypothesis on the estimated system
rls_params.properties = ["Strictly proper"];

%% Predictive Cost Adaptive Control (PCAC) parameters
pcac_params.nb_sample = 500; % Int > 0

% Input constraints
pcac_params.u_min = -1*[1;1;1]; % Scalar
pcac_params.u_max = 1*[1;1;1]; % Scalar
pcac_params.delta_u_min = -0.5*1*[1;1;1]; % Scalar
pcac_params.delta_u_max = 0.5*1*[1;1;1]; % Scalar

pcac_params.l = 40; % Horizon  % int > 0
pcac_params.Q_bar = 40*eye(3); % Tracking error cost % Scalar >= 0 for 1-dim error, matrix >=0 for n-dim error
pcac_params.P_bar = 40*eye(3); % Terminal tracking error cost  % Scalar >= 0 for 1-dim error, matrix >= 0 for n-dim error
pcac_params.R = 1^2*eye(3); % Rate input cost  % Scalar >= 0 for 1-dim input, matrix >= 0 for n-dim input
pcac_params.S = [];%10*eye(size(sys_params.C,1));%*eye(pcac_params.l);

%% DeePC Parameters

deepc_params.max_data = 100;
deepc_params.T_d = 10;
deepc_params.nb_try = 2;
deepc_params.T_ini = 2;%rls_params.n_est;
deepc_params.T_f = 20;%pcac_params.l;
deepc_params.lambda_y_ini = 100;
deepc_params.lambda_u_ini = 100;
deepc_params.lambda_g = 0.1;
deepc_params.h = @(x) x'*x;
deepc_params.Proj_norm = 1;

%%
params.sys_params = sys_params;
params.rls_params = rls_params;
params.pcac_params = pcac_params;
params.deepc_params = deepc_params;