function x_1k = one_step_prediction(idx, Y, U, theta, params)
% One step prediction (PCAC paper (23)-(28))

rls_params = params.rls_params;
n_est = rls_params.n_est;

sys_params = params.sys_params;
n_y = sys_params.n_y;
n_u = sys_params.n_u;

x_hat_k = zeros(n_y,n_est);

theta_mat = reshape(theta,n_y,[]); % [F_1,...,F_n,G_0,...,G_n]

x_hat_k(:,1) = Y(:,idx) - theta_mat(:,n_est*n_y+1:n_est*n_y+n_u)*U(:,idx); % y(k) - G_0 u(k) (Eq 25)

for i=2:n_est
    x_hat_k(:,i) = -theta_mat(:,(i-1)*n_y+1:n_est*n_y)*reshape(Y(:,idx-1:-1:idx-(n_est-i+1)),[],1) +....
                    theta_mat(:,n_est*n_y + (i*n_u+1:(n_est+1)*n_u))*reshape(U(:,idx-1:-1:idx-(n_est-i+1)),[],1);  % Eq 26
end

[A_k,B_k,C_k,D_k] = pcac_matrices_representation_1(theta,params); % Eqs 27,28
x_1k = A_k*reshape(x_hat_k,[],1) + B_k*U(:,idx); % Eq 23

end

