function u_pcac = pcac(idx, Y, U, theta, disturbance, params)

%% Compute x_1|k and u_k = U(k)
x_1 = one_step_prediction(idx, Y, U+disturbance, theta, params);

u_k = U(:,idx);

%% PCAC matrices from Theta
[A_k,B_k,C_k,D_k] = pcac_matrices(theta,params);

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
R_kl = kron(ones(l,1),ref(idx));

%% RLS parameters
rls_params = params.rls_params;
n_est = rls_params.n_est;

%% Gamma and T
Gamma = zeros(l*n_y,n_est*n_y);
T = zeros(l*n_y,l*n_u);
T = kron(eye(l),D_k);

for k=0:l-1
    Gamma(k*n_y+1:(k+1)*n_y,1:n_est*n_y) = C_k*A_k^k;
end

for k=1:l-1
    T(k*n_y+1:end,(k-1)*n_u+1:k*n_u) = Gamma(1:(l-k)*n_y,:)*B_k;
end

%% C_tl, C_l and D_l
C_tl = kron(eye(l),C_t);
C_l = kron(eye(l),C*C_c);
D_l = kron(ones(l,1),D);

%% U_min/max and Delta_U_min/max
U_min = kron(ones(l,1),u_min);U_max = kron(ones(l,1),u_max);
DeltaU_min = kron(ones(l,1),delta_u_min);DeltaU_max = kron(ones(l,1),delta_u_max);

Q = blkdiag(Q_bar,P_bar);

Grad_mat = eye(l*n_u) - diag(ones(1,(l-1)*n_u),-n_u);
Grad_u = [-eye(n_u);zeros((l-1)*n_u,n_u)];

H = blkdiag(C_t'*Q*C_t,Grad_mat'*R*Grad_mat,zeros(n_u*l));
f = -[(Q*C_t)'*R_kl;zeros(2*n_u*l,1)];
A = [C_l,zeros(size(C_l,1),n_u*l),zeros(size(C_l,1),n_u*l)];
b = -D_l;

mat_bounds = blkdiag(eye(l*n_u),inv(Grad_mat));
lb_u = mat_bounds*([U_min;DeltaU_min] - [zeros(l*n_u,1);Grad_u*u_k]);
ub_u = mat_bounds*([U_max;DeltaU_max] - [zeros(l*n_u,1);Grad_u*u_k]);
LB = [-inf+zeros(n_y*l,1);lb_u];
UB = [inf+zeros(n_y*l,1);ub_u];

opts = optimset("Display","None");

H_tot = H;
H_tot = 1/2*(H_tot+H_tot');
A_ineq = A;
B_ineq = b;
A_eq = [eye(n_y*l),-T,zeros(n_y*l,n_u*l);
        zeros(n_u*l,n_y*l),eye(n_u*l),-eye(n_u*l)];
B_eq = [Gamma*x_1;
        zeros(n_u*l,1)];
lb_tot = LB;
ub_tot = UB;
U_pcac = quadprog(H_tot,f,A_ineq,B_ineq,A_eq,B_eq,lb_tot,ub_tot,[],opts); 

% A_ineq = [A;
%           eye(n_u*l);
%           -eye(n_u*l);
%           Grad_mat;
%           -Grad_mat];
% B_ineq = [b;
%           U_max;
%           -U_min;
%           DeltaU_max - Grad_u*u_k;
%           -(DeltaU_min - Grad_u*u_k)];
% U_pcac = quadprog(H,f,A_ineq,B_ineq,[],[],[],[],[],opts);
if isempty(U_pcac)
    u_pcac = 0;
else
    u_pcac = U_pcac(n_y*l + (1:n_u));
    %u_pcac = U_pcac((1:n_u));
end

end


