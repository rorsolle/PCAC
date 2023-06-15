function [Theta,P] = rls_code(idx, Y, U, P, Theta, params)
% Recursive Least Squares Online Identification

rls_params = params.rls_params;
t_d = rls_params.t_d;
t_n = rls_params.t_n;
n_est = rls_params.n_est;
eta = rls_params.eta;

sys_params = params.sys_params;
n_y = sys_params.n_y;
n_u = sys_params.n_u;

if isfield(rls_params,"properties") && any("Strictly proper"==rls_params.properties)
    Theta(n_y*n_y*n_est+1:n_y*n_y*n_est+n_y*n_u,:) = 0; %Set G_0 to 0
end

% Without VRF
if isfield(rls_params,"lambda")
    lambda = rls_params.lambda;

% With VRF
else
    if idx - t_d > 0
        Z = zeros(n_y,t_d+1);
        count = 0;
        for k=idx-t_d:idx
            count=count+1;
            phi_k = compute_phi(Y, U, k, params);
            Z(:,count) = Y(:,k) - phi_k*Theta(:,idx);
        end
        g = vrf(Z,params);
        lambda = 1/(1+eta*g*(g>=0));
    else
        lambda = 1;
    end
end

phi_k = compute_phi(Y, U, idx, params);
z_k = Y(:,idx) - phi_k*Theta(:,idx);

L_k = 1/lambda*P(:,:,idx);
P(:,:,idx+1) = L_k - L_k*phi_k'*((eye(n_y) + phi_k*L_k*phi_k')\phi_k*L_k);
Theta(:,idx+1) = Theta(:,idx) + P(:,:,idx+1)*phi_k'*z_k;

if isfield(rls_params,"properties") && any("Strictly proper"==rls_params.properties)
    Theta(n_y*n_y*n_est+1:n_y*n_y*n_est+n_y*n_u,:) = 0;
end


end

