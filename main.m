clear; close all;

%% Load parameters and model
%Example_S_5
%Example_S_6
Example_1
%Example_2

params.sys_params = sys_params;
params.rls_params = rls_params;
params.pcac_params = pcac_params;

nb_sample = 60;

%% Data to store
Y = zeros(n_y,nb_sample);
U = zeros(n_u,nb_sample);

nb_var = n_est*n_y*(n_u+n_y)+n_u*n_y; %Size of theta
Theta = zeros(nb_var,nb_sample+1);
Theta(:,1) = Theta_0.*ones(nb_var,1);
P = zeros(nb_var,nb_var,nb_sample+1);
P(:,:,1) = P_0.*eye(nb_var,nb_var);

%% Compute the theta reference
[num,den] = tfdata(G);
num = num{1};den = den{1};
theta_ref = [-den(end:-1:2),num]';

%% Noise
W = randn(n_u,nb_sample)*std_w;
V = randn(n_y,nb_sample)*std_v;

t = 0:1:nb_sample-1;

%% PCAC + RLS
for k=1:nb_sample-1
    [Theta,P] = rls_code(k, Y + V, U, P, Theta, params); %RLS
    if k>n_est
    u_pcac = pcac(k, Y, U, Theta(:,k), params); %PCAC
    U(:,k+1) = u_pcac;
    end
    Y = lsim(G,(U+W)',t)'; %System reponse
end
[Theta,P] = rls_code(k+1, Y + V, U, P, Theta, params); %RLS

plot_results