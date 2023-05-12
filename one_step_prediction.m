function x_1k = one_step_prediction(idx, Y, U, theta, params)

rls_params = params.rls_params;
n_est = rls_params.n_est;

sys_params = params.sys_params;
n_y = sys_params.n_y;
n_u = sys_params.n_u;

x_hat_k = zeros(n_y,n_est);


theta_mat = reshape(theta,n_y,[]);

x_hat_k(:,1) = Y(:,idx) - theta_mat(:,n_est*n_y+1:n_est*n_y+n_u)*U(:,idx);

for i=2:n_est
    x_hat_ik = 0;
    for j=1:n_est-i+1
        index_mat = i+j-1; 
        F = theta_mat(:,(index_mat-1)*n_y+1:index_mat*n_y);
        G = theta_mat(:,n_est*n_y+(index_mat-1)*n_u+1:n_est*n_y+index_mat*n_u);
        x_hat_ik = x_hat_ik - F*Y(:,idx-j) + G*U(:,idx-j);
    end
    x_hat_k(:,i) = x_hat_ik;
end

[A_k,B_k,C_k,D_k] = pcac_matrices(theta,params);
x_1k = A_k*reshape(x_hat_k,[],1) + B_k*U(:,idx);

end

