function u_pcac = pcac_normal(idx, Y, U, theta, params)

n_est = params.rls_params.n_est;

%% Compute x_1|k and u_k = U(k)
%x_0 = one_step_prediction(idx-1, Y, U, theta, params);
%x_1 = one_step_prediction(idx, Y, U, theta, params);
x_1 = Y(:,idx:-1:idx-n_est+1)';

U_k = U(:,idx-1:-1:idx-n_est)';

%% PCAC matrices from Theta
[A_k,B_k,C_k,D_k] = pcac_matrices_representation_2(theta,params);

%% PCAC parameters
pcac_params = params.pcac_params;
l = pcac_params.l;
Q_bar = pcac_params.Q_bar;
P_bar = pcac_params.P_bar;
R = pcac_params.R;
u_min = pcac_params.u_min;u_max = pcac_params.u_max;
delta_u_min = pcac_params.delta_u_min;delta_u_max = pcac_params.delta_u_max;

%% System parameters
sys_params = params.sys_params;
n_y = sys_params.n_y;
n_u = sys_params.n_u;
C_t = sys_params.C_t;
C_c = sys_params.C_c;
C = sys_params.C;
D = sys_params.D;
ref = sys_params.ref;
r_kl = ref(idx);

n_r = size(r_kl,1);

%% RLS parameters
rls_params = params.rls_params;
n_est = rls_params.n_est;

%% Gamma and T
% X = [x , err  , U, ref]
% X(k+1) = A_tilde X(k) + B_tilde Delta U 
% y(k) = C_tilde X(k)

A_tilde = [A_k                        , zeros(n_y*n_est,n_r) , B_k                  , zeros(n_y*n_est,n_r);
           C_t*C_k*A_k                , zeros(n_r)           , C_t*C_k*B_k          , -eye(n_r);
           zeros(n_u*n_est,n_y*n_est) , zeros(n_u*n_est,n_r) , eye(n_u*n_est)       , zeros(n_u*n_est,n_r);
           zeros(n_r,n_y*n_est)       , zeros(n_r,n_r)       , zeros(n_r,n_u*n_est) , eye(n_r,n_r)];

B_tilde = [B_k;%zeros(n_y*n_est,n_u*n_est);
            C_t*C_k*B_k;%zeros(n_r,n_u*n_est);
           eye(n_u*n_est);
           zeros(n_r,n_u*n_est)];

C_tilde = C_k*[eye(n_y*n_est),zeros(n_y*n_est,n_r+n_u*n_est+n_r)];

A_ineq_y = C*C_c;
B_ineq_y = -D;
A_ineq_u = 0*ones(1,n_u*n_est);
B_ineq_u = 0;

%% U_min/max and Delta_U_min/max

lb_x = [-inf+zeros(n_y*n_est,1);
        -inf+zeros(n_r,1);
        u_min*ones(n_est*n_u,1);
        -inf+zeros(n_r,1)];

ub_x = [inf+zeros(n_y*n_est,1);
        inf+zeros(n_r,1);
        u_max*ones(n_est*n_u,1);
        inf+zeros(n_r,1)];

lb_u = delta_u_min*ones(n_est*n_u,1);
ub_u = delta_u_max*ones(n_est*n_u,1);

x0 = [x_1;C_t*C_k*x_1-r_kl;U_k;r_kl];

Q = blkdiag(zeros(n_y*n_est), ...
            Q_bar, ...
            zeros(n_u*n_est), ...
            zeros(n_r));

P = blkdiag(zeros(n_y*n_est), ...
            P_bar, ...
            zeros(n_u*n_est), ...
            zeros(n_r));

R = diag(R/n_est*ones(n_est,1));

is_ordered_input  = 1;
u = MPC(A_tilde,B_tilde,C_tilde, ...
    x0, ...
    A_ineq_y,B_ineq_y, ...
    A_ineq_u,B_ineq_u, ...
    lb_x,ub_x, ...
    lb_u,ub_u, ...
    Q,P,R, ...
    l, ...
    is_ordered_input);

u_pcac = U(:,idx-1) + u(1);
end
