function [A_k,B_k,C_k,D_k] = pcac_matrices_representation_1(theta,params)
% Representation equation (23)->(31)

sys_params = params.sys_params;
n_y = sys_params.n_y;
n_u = sys_params.n_u;

rls_params = params.rls_params;
n_est = rls_params.n_est;

theta_mat = reshape(theta,n_y,[]);

A_k = zeros(n_y*n_est,n_y*n_est);
B_k = zeros(n_y*n_est,n_u);
C_k = zeros(n_y,n_y*n_est);
D_k = theta_mat(:,n_y*n_est+1:n_y*n_est+n_u);

G_0 = theta_mat(1:n_y,n_est*n_y+1:n_est*n_y+n_u);
for k=1:n_est
    F_k = theta_mat(1:n_y,(k-1)*n_y+1:k*n_y);
    G_k = theta_mat(1:n_y,n_est*n_y+k*n_u+1:n_est*n_y+(k+1)*n_u);

    A_k((k-1)*n_y+1:k*n_y,1:n_y) = -F_k;
    B_k((k-1)*n_y+1:k*n_y,1:n_u) = G_k - F_k*G_0;
end

A_k = A_k + [zeros(n_y*(n_est-1),n_y),eye(n_y*(n_est-1));zeros(n_y,n_y*n_est)];

C_k = [eye(n_y),zeros(n_y,n_y*(n_est-1))];
D_k = theta_mat(1:n_y,n_est*n_y+1:n_est*n_y+n_u);


end
