function [t,Y,U] = odeepc_run(Y,V,U,W,params)

sat = @(x) min(max(x,params.pcac_params.u_min),params.pcac_params.u_max);
sat_delta = @(dx) min(max(dx,params.pcac_params.delta_u_min),params.pcac_params.delta_u_max);

sys_params = params.sys_params;
rls_params = params.rls_params;
pcac_params = params.pcac_params;
deepc_params = params.deepc_params;
odeepc_params = params.odeepc_params;

p = sys_params.n_y;
m = sys_params.n_u;

kappa = odeepc_params.kappa;
T_ini = deepc_params.T_ini;
N = deepc_params.T_f; % Prediction horizon

y_tau = reshape(Y(:,1:N),[],1);
u_tau = reshape(U(:,1:N),[],1);
g_tau = zeros(kappa,1);
v_tau = zeros((p+m)*(T_ini+N),1);

tau_var.y_tau = y_tau;
tau_var.u_tau = u_tau;
tau_var.g_tau = g_tau;
tau_var.v_tau = v_tau;

Ts = params.sys_params.Ts;

t = Ts*(0:params.pcac_params.nb_sample-1);
x = params.sys_params.x0;

for k=1:params.pcac_params.nb_sample-1
    Y(:,k) = measure_fct(t(k),x,U(:,k)+W(:,k),params) + V(:,k);

    if k>params.deepc_params.T_ini+10
        [u_deepc, tau_var] = odeepc(k, Y + V, U, tau_var, params); %PCAC
        U(:,k+1) = sat(u_deepc);
        %U(:,k+1) = sat(U(:,k) + sat_delta(u_deepc-U(:,k)));
    else
        U(:,k+1) = sat((params.pcac_params.u_max - params.pcac_params.u_min).*rand(params.sys_params.n_u,1) + params.pcac_params.u_min);
    end
    x = simu_fct(linspace(t(k),t(k+1),2), x,U(:,k)+W(:,k),params); %System reponse
end
Y(:,k+1) = measure_fct(t(k+1),x,U(:,k+1)+W(:,k+1),params) + V(:,k+1);

end

