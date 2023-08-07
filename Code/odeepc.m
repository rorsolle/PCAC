function [u_deepc, new_tau_var] = odeepc(idx, Y, U, tau_var, params)

sys_params = params.sys_params;
rls_params = params.rls_params;
pcac_params = params.pcac_params;
deepc_params = params.deepc_params;
odeepc_params = params.odeepc_params;

y_tau = tau_var.y_tau;
u_tau = tau_var.u_tau;
g_tau = tau_var.g_tau;
v_tau = tau_var.v_tau;

p = sys_params.n_y;
m = sys_params.n_u;

Q_bar = pcac_params.Q_bar;
P_bar = pcac_params.P_bar;
R = pcac_params.R;

T_ini = deepc_params.T_ini;
N_l = odeepc_params.N_l;
kappa = odeepc_params.kappa;
alpha = odeepc_params.alpha;

N = deepc_params.T_f; % Prediction horizon
T_tot = T_ini + N;

eps_g = odeepc_params.eps_g;
eps_v = odeepc_params.eps_v;

% U_p = mT_ini x kappa
% U_f = mN x kappa
y_ini_t = reshape(Y(:,idx-T_ini+1:1:idx),[],1);
u_ini_t = reshape(U(:,idx-T_ini+1:1:idx),[],1);

%N_inter = min(N, idx - T_ini);
N_inter = N;

if idx-(T_ini+N)+1<=kappa
    u_deepc = (pcac_params.u_max-pcac_params.u_min).*rand(m,1) + pcac_params.u_min;
else
    y_ref = reshape(sys_params.ref(idx:idx+N-1),[],1);

    size_data = min(idx-(T_ini+N_inter)+1,kappa);
    
    data_Y = Y(:,idx-(size_data+T_ini+N)+1:idx-1);
    data_U = U(:,idx-(size_data+T_ini+N)+1:idx-1);
    [Y_p,Y_f] = hankel_mat(data_Y, T_ini, N_inter);
    [U_p,U_f] = hankel_mat(data_U, T_ini, N_inter);
    
    U_t = [U_p;U_f];
    Y_t = [Y_p;Y_f];
    H_t = [U_t;
           Y_t];
    h_t = [u_ini_t;u_tau(1:m*N_inter);y_ini_t;y_tau(1:p*N_inter)];

    Q = blkdiag(kron(eye(N-1),Q_bar),P_bar);
    R = kron(eye(N),R);

    for tau=1:N_l
    
        if mod(tau,N_l)>= 1
            
            grad_y_f = 2*Q*(y_tau - y_ref);
            grad_u_f = 2*R*u_tau;
            
            v_u_tau = v_tau(1:m*(T_ini+N));
            v_y_tau = v_tau(m*(T_ini+N)+1:(m+p)*(T_ini+N));
            index_v_tau = ([1:m*(T_ini+N_inter),m*(T_ini+N)+1:m*(T_ini+N)+p*(T_ini+N_inter)]);
            
            % Equations 9a-9d
            new_u_tau = Proj_u(u_tau - alpha*(grad_u_f - v_u_tau(m*T_ini+1:end)),params);
            new_y_tau = Proj_y(y_tau - alpha*(grad_y_f - v_y_tau(p*T_ini+1:end)),params);
            new_g_tau = g_tau - alpha*(H_t.'*v_tau(index_v_tau) + eps_g*g_tau);
            new_v_tau = v_tau - eps_v*v_tau;
            new_v_tau(index_v_tau) = new_v_tau(index_v_tau) + alpha*(H_t*g_tau - h_t);
        
            u_tau = new_u_tau;
            y_tau = new_y_tau;
            g_tau = new_g_tau;
            v_tau = new_v_tau;
        
        else
            
            u_hat_tau = shift_mat(N,m)*u_tau;
            y_hat_tau = shift_mat(N,p)*y_tau;
            v_u_hat_tau = shift_mat(T_tot,m)*v_u_tau;
            v_y_hat_tau = shift_mat(T_tot,p)*v_y_tau;
            v_hat_tau = [v_u_hat_tau;
                         v_y_hat_tau];
        
            u_0_tau = u_tau(1:m);
            u_1_tau = u_tau(m+1:2*m);
            y_0_tau = y_tau(1:p);
            u_ini_tp1 = shift_mat(T_ini,m)*u_ini_t + [zeros(m,m*(T_ini-1)),eye(m)].'*u_0_tau;
            y_ini_tp1 = shift_mat(T_ini,p)*y_ini_t + [zeros(p,p*(T_ini-1)),eye(p)].'*y_0_tau;
        
            % Equations 11a-11b
            grad_y_hat_f = 2*Q*(y_hat_tau - y_ref);
            grad_u_hat_f = 2*R*u_hat_tau;

            u_taup1 = Proj_u(u_hat_tau - alpha*(grad_u_hat_f - v_u_hat_tau(m*T_ini+1:m*(T_ini+N))),params);
            y_taup1 = Proj_u(y_hat_tau - alpha*(grad_y_hat_f - v_y_hat_tau(p*T_ini+1:p*(T_ini+N))),params);
        
            % Equations 12a-12c
            h_tp1 = [u_ini_tp1 ; u_taup1;
                     y_ini_tp1 ; y_taup1];
        
            H_add_t = [zeros(m*(T_ini+N_inter-1),kappa);
                       [zeros(m,m*(T_ini+N_inter-1)),eye(m)]*U_t*shift_mat(kappa,1).' + [zeros(m,kappa-1),[eye(m),zeros(m,m*(T_ini-1))]*u_ini_t];
                       zeros(p*(T_ini+N_inter-1),kappa);
                       [zeros(p,p*(T_ini+N_inter-1)),eye(p)]*Y_t*shift_mat(kappa,1).' + [zeros(p,kappa-1),[eye(p),zeros(p,p*(T_ini-1))]*y_ini_t];
                       ];
        
            H_tp1 = blkdiag(shift_mat(T_tot,m),shift_mat(T_tot,p))*H_t + H_add_t;

            % Equations 11c-11d
            g_taup1 = g_tau - alpha*(H_tp1.'*v_hat_tau + eps_g*g_tau);
            v_taup1 = v_hat_tau + alpha*(H_tp1*g_tau - h_tp1 - eps_v*v_hat_tau);
        
            y_tau = y_taup1;
            u_tau = u_taup1;
            g_tau = g_taup1;
            v_tau = v_taup1;
        end
    end
    u_deepc = u_0_tau(1:m);
    %u_deepc = u_1_tau(1:m);
end

new_tau_var.y_tau = y_tau;
new_tau_var.u_tau = u_tau;
new_tau_var.g_tau = g_tau;
new_tau_var.v_tau = v_tau;

end

function u_proj = Proj_u(u, params)
    pcac_params = params.pcac_params;
    u_min = pcac_params.u_min;
    u_max = pcac_params.u_max;
    du_min = pcac_params.delta_u_min;
    du_max = pcac_params.delta_u_max;

    u_sat = min(u_max,max(u,u_min));
    du = diff(u_sat,1,2);
    du_sat = min(du_max,max(du,du_min));
    u_proj = cumsum([u_sat(:,1),diff(du_sat,1,2)],2);
end

function y_proj = Proj_y(y, params)
    y_proj = y;
end

function S = shift_mat(m,n)
    S = kron(diag(ones(m-1,1),1),eye(n));
end