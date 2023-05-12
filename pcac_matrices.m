function [A_k,B_k,C_k,D_k] = pcac_matrices(theta,params)

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

A_test_k = zeros(n_y*n_est,n_y*n_est);
B_test_k = zeros(n_y*n_est,n_u*n_est);

for k=1:n_est
    F_k = theta_mat(1:n_y,(k-1)*n_y+1:k*n_y);
    G_k = theta_mat(1:n_y,n_est*n_y+(k-1)*n_u+1:n_est*n_y+k*n_u);
    G_k_1 = theta_mat(1:n_y,n_est*n_y+k*n_u+1:n_est*n_y+(k+1)*n_u);

    A_k((k-1)*n_y+1:k*n_y,1:n_y) = -F_k;
    B_k((k-1)*n_y+1:k*n_y,1:n_u) = G_k_1 - F_k*G_k;
end

A_test_k = [-theta_mat(1:n_y,1:n_est*n_y);
            eye(n_y*(n_est-1)),zeros(n_y*(n_est-1),n_y)];
B_test_k = [theta_mat(1:n_y,n_est*n_y+n_u+1:n_est*n_y+(n_est+1)*n_u);
            zeros(n_y*(n_est-1),n_u*n_est)];

A_k = A_k + [zeros(n_y*(n_est-1),n_y),eye(n_y*(n_est-1));zeros(n_y,n_y*n_est)];
C_k = [eye(n_y),zeros(n_y,n_y*(n_est-1))];
D_k = theta_mat(1:n_y,n_est*n_y+1:n_est*n_y+n_u);

A_k = A_test_k;
B_k = B_test_k;

end
