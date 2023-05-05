function phi = compute_phi(Y, U, idx, params)

sys_params = params.sys_params;
n_y = sys_params.n_y;

rls_params = params.rls_params;
n_est = rls_params.n_est;

Y_vec = reshape(-Y(:,idx-1:-1:idx-n_est),1,[]);
U_vec = reshape(U(:,idx:-1:idx-n_est),1,[]);
phi = kron([Y_vec,U_vec],eye(n_y));

end

