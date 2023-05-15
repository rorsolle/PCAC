function [A_k,B_k,C_k,D_k] = pcac_matrices_representation_2(theta,params)
% Representation [y(k),...,y(k_n+1)]^T = A_k[y(k-1),...,y(k_n)]^T +
%                                        B_k[u(k-1),...,u(k_n)]^T
% WARNING : Works only for stricly proper systems

sys_params = params.sys_params;
n_y = sys_params.n_y;
n_u = sys_params.n_u;

rls_params = params.rls_params;
n_est = rls_params.n_est;

theta_mat = reshape(theta,n_y,[]);

A_k = [-theta_mat(1:n_y,1:n_est*n_y);
            eye(n_y*(n_est-1)),zeros(n_y*(n_est-1),n_y)];

B_k = [theta_mat(1:n_y,n_est*n_y+n_u+1:n_est*n_y+(n_est+1)*n_u);
            zeros(n_y*(n_est-1),n_u*n_est)];

C_k = [eye(n_y),zeros(n_y,n_y*(n_est-1))];

D_k = zeros(n_y,n_est*n_u);


end
