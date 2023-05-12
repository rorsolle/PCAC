clear; close all;

%% Load parameters and model
%Example_S_5
%Example_S_6
Example_1
%Example_2

params.sys_params = sys_params;
params.rls_params = rls_params;
params.pcac_params = pcac_params;

nb_sample = 100;

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
theta_ref = [den(2:end),num]';

%% Noise
W = randn(n_u,nb_sample)*std_w;
V = randn(n_y,nb_sample)*std_v;
U = U+W;
t = 0:1:nb_sample-1;

sat = @(x) min(max(x,pcac_params.u_min),pcac_params.u_max);
%% PCAC + RLS
disturbance=zeros(nb_sample,1);
L = 0*ones(1,n_y);
for k=1:nb_sample-1
    [Theta,P] = rls_code(k, Y + V, U, P, Theta, params); %RLS
    if k>n_est
    %u_pcac = pcac(k, Y, U, disturbance(k), Theta(:,k), params); %PCAC
    u_pcac = test_pcac_vel(k, Y + V, U, Theta(:,k+1), params); %PCAC
    U(:,k+1) = sat(u_pcac + randn(n_u,1)*std_w);
    end
    
    Y = lsim(G,(U)',t)'; %System reponse
%     if k>n_est-1
%         x_1k = one_step_prediction(k, Y, U, Theta(:,k), params);
%     else
%         x_1k = zeros(n_est*n_y,1);
%     end
%     [A_k,B_k,C_k,D_k] = pcac_matrices(Theta(:,k),params);
%     y_est = C_k*x_1k + D_k*U(:,k+1);
%     disturbance(k+1) = disturbance(k) + L*(Y(:,k+1) - y_est);
end
[Theta,P] = rls_code(k+1, Y + V, U, P, Theta, params); %RLS

plot_results
