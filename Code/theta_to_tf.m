function G_tf = theta_to_tf(theta, z, params)

sys_params = params.sys_params;
n_y = sys_params.n_y;
n_u = sys_params.n_u;

rls_params = params.rls_params;
n_est = rls_params.n_est;

theta_mat = reshape(theta,n_y,[]);

G_0 = theta_mat(1:n_y,n_est*n_y+1:n_est*n_y+n_u);

sum_U = z^(n_est)*G_0;

sum_Y = z^(n_est)*eye(n_y);

for k=1:n_est
    F_k = theta_mat(1:n_y,(k-1)*n_y+1:k*n_y);
    G_k = theta_mat(1:n_y,n_est*n_y+k*n_u+1:n_est*n_y+(k+1)*n_u);
    
    sum_Y = sum_Y + z^(n_est-k)*F_k;
    sum_U = sum_U + z^(n_est-k)*G_k;
end

G_tf = sum_Y\sum_U;

end