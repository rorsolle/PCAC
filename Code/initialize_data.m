function [Y,U,V,W,Theta,P] = initialize_data(params)
% Initialization of datasets

sys_params = params.sys_params;
rls_params = params.rls_params;
pcac_params = params.pcac_params;

%% Data to store
Y = zeros(sys_params.n_y,pcac_params.nb_sample);
U = zeros(sys_params.n_u,pcac_params.nb_sample);

Theta = zeros(params.nb_var,pcac_params.nb_sample+1);
Theta(:,1) = rls_params.Theta_0.*ones(params.nb_var,1);

P = zeros(params.nb_var,params.nb_var,pcac_params.nb_sample+1);
P(:,:,1) = rls_params.P_0*eye(params.nb_var,params.nb_var);

%% Noise
W = sys_params.std_w*randn(sys_params.n_u,pcac_params.nb_sample);
V = sys_params.std_v*randn(sys_params.n_y,pcac_params.nb_sample);

end

