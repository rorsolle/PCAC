function u_pcac = pcac_paper(idx, Y, U, theta, params)

%% Compute x_1|k and u_k = U(k)
x_1 = one_step_prediction(idx, Y, U, theta, params);

u_k = U(:,idx);

%% PCAC matrices from Theta
[A_k,B_k,C_k,D_k] = pcac_matrices_representation_1(theta,params);

%% PCAC parameters
pcac_params = params.pcac_params;
l = pcac_params.l;
Q_bar = pcac_params.Q_bar;
P_bar = pcac_params.P_bar;
R = pcac_params.R;
S = pcac_params.S;
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
n_s = size(S,1);

%% RLS parameters
rls_params = params.rls_params;
n_est = rls_params.n_est;

%% Gamma and T
% X = [x , err  , ref, slack, U]
% X(k+1) = A_tilde X(k) + B_tilde [Delta Slack; Delta U] 
% y(k) = C_tilde X(k)


A_tilde = [A_k                        , zeros(n_y*n_est,n_r) , zeros(n_y*n_est,n_r) , zeros(n_y*n_est,n_s) , B_k ;
           C_t*C_k*A_k                , zeros(n_r)           , -eye(n_r)            , zeros(n_r,n_s)       , C_t*C_k*B_k;
           zeros(n_r,n_y*n_est)       , zeros(n_r,n_r)       , eye(n_r,n_r)         , zeros(n_r,n_s)       , zeros(n_r,n_u);
           zeros(n_s,n_y*n_est)       , zeros(n_s,n_r)       , zeros(n_s,n_r)       , eye(n_s,n_s)         , zeros(n_s,n_u);
           zeros(n_u,n_y*n_est)       , zeros(n_u,n_r)       , zeros(n_u,n_r)       , zeros(n_u,n_s)       , eye(n_u,n_u);
];

B_tilde = [zeros(n_y*n_est,n_s) , B_k;
           zeros(n_r,n_s) , C_t*C_k*B_k;
           zeros(n_r,n_s+n_u);
           eye(n_s)       , zeros(n_s,n_u);
           zeros(n_u,n_s) , eye(n_u)];

%C_tilde = C_k*[eye(n_y*n_est),zeros(n_y*n_est,n_r+n_u+n_r)];
C_tilde = [C_k,zeros(size(C_k,1),n_r+n_r+n_s+n_u);
           zeros(n_s,n_y*n_est+n_r+n_r),eye(n_s),zeros(n_s,n_u)];

A_ineq_y = [C*C_c,-eye(n_s)];
B_ineq_y = -D;
A_ineq_u = 0*ones(1,n_s+n_u);
B_ineq_u = 0;

%% U_min/max and Delta_U_min/max

lb_x = [-inf+zeros(n_y*n_est,1);
        -inf+zeros(n_r,1);
        -inf+zeros(n_r,1);
        zeros(n_s,1);
        u_min];

ub_x = [inf+zeros(n_y*n_est,1);
        inf+zeros(n_r,1);
        inf+zeros(n_r,1)
        inf+zeros(n_s,1)
        u_max];

lb_u = [-inf+zeros(n_s,1);
        delta_u_min];
ub_u = [inf+zeros(n_s,1);
        delta_u_max];

x0 = [x_1;C_t*C_k*x_1-r_kl;r_kl;zeros(n_s,1);u_k];

Q = blkdiag(zeros(n_y*n_est), ...
            Q_bar, ...
            zeros(n_r), ...
            S, ...
            zeros(n_u));

P = blkdiag(zeros(n_y*n_est), ...
            P_bar, ...
            zeros(n_r), ...
            S, ...
            zeros(n_u));

R = blkdiag(zeros(n_s),R);

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

u_pcac = U(:,idx) + u(n_s+1:end);

end


