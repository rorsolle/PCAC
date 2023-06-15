function phi = compute_phi(Y, U, idx, params)
% Compute phi(k) = [-y(k-1)^T,...,-y(k-nhat)^T,u(k)^T,...,u(k-n)^T] x I_p

sys_params = params.sys_params;
n_y = sys_params.n_y;

rls_params = params.rls_params;
n_est = rls_params.n_est;

if (idx - n_est > 0)
    Y_vec = reshape(-Y(:,idx-1:-1:idx-n_est),1,[]);
    U_vec = reshape(U(:,idx:-1:idx-n_est),1,[]);
    phi = kron([Y_vec,U_vec],eye(n_y));
else
    Y_vec = reshape(-[Y(:,idx-1:-1:1),kron(ones(1,n_est-idx+1),Y(:,1))],1,[]);
    U_vec = reshape([U(:,idx:-1:1),kron(ones(1,n_est-idx+1),U(:,1))],1,[]);
    phi = kron([Y_vec,U_vec],eye(n_y));
end

