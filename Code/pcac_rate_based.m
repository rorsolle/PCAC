function u_pcac = pcac_rate_based(idx, Y, U, theta, params)

n_est = params.rls_params.n_est;

%% Compute x_1|k and u_k = U(k)
x_0 = one_step_prediction(idx-1, Y, U, theta, params);
x_1 = one_step_prediction(idx, Y, U, theta, params);

U_k = U(:,idx);

%% PCAC matrices from Theta
[A_k,B_k,C_k,D_k] = pcac_matrices_representation_1(theta,params);
x_1 = A_k*x_0 + B_k*U_k;

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

%% Rate-based system 
% X = [Delta x , err , x , U]
% X(k+1) = A_tilde X(k) + B_tilde Delta U 
% y(k) = C_tilde X(k)

A_tilde = [A_k                        , zeros(n_y*n_est,n_r) , zeros(n_y*n_est,n_y*n_est) , zeros(n_y*n_est,n_u);
           C_t*C_k*A_k                , eye(n_r)             , zeros(n_r,n_y*n_est)       , zeros(n_r,n_u);
           eye(n_y*n_est)             , zeros(n_y*n_est,n_r) , eye(n_y*n_est)             , zeros(n_y*n_est,n_u);
           zeros(n_u,n_y*n_est)       , zeros(n_u,n_r)       , zeros(n_u,n_y*n_est)       , eye(n_u)];

B_tilde = [B_k;
           C_t*C_k*B_k;
           zeros(n_y*n_est,n_u);
           eye(n_u)];

C_tilde = C_k*[zeros(n_y*n_est),zeros(n_y*n_est,n_r),eye(n_y*n_est),zeros(n_y*n_est,n_u)];

A_ineq_y = C*C_c;
B_ineq_y = -D;
A_ineq_u = 0*ones(1,n_u);
B_ineq_u = 0;

%% U_min/max and Delta_U_min/max

lb_x = [-inf+zeros(n_y*n_est,1);
        -inf+zeros(n_r,1);
        -inf+zeros(n_y*n_est,1);
        u_min;    
        ];

ub_x = [inf+zeros(n_y*n_est,1);
        inf+zeros(n_r,1);
        inf+zeros(n_y*n_est,1)
        u_max;
        ];

lb_u = delta_u_min;
ub_u = delta_u_max;

x0 = [x_1-x_0;C_t*C_k*x_1-r_kl;x_1;U_k];

Q = blkdiag(zeros(n_y*n_est), ...
            Q_bar, ...
            zeros(n_y*n_est), ...
            zeros(n_u));

P = blkdiag(zeros(n_y*n_est), ...
            P_bar, ...
            zeros(n_y*n_est), ...
            zeros(n_u));

%R = kron(eye(n_est),R/n_est);

is_ordered_input  = 0;
is_diff_input = 0;
u = MPC(A_tilde,B_tilde,C_tilde, ...
        x0, ...
        A_ineq_y,B_ineq_y, ...
        A_ineq_u,B_ineq_u, ...
        lb_x,ub_x, ...
        lb_u,ub_u, ...
        Q,P,R, ...
        l, ...
        is_ordered_input, ...
        is_diff_input);


u_pcac = U(:,idx) + u(1);

end
