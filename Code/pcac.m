function [t,Y,U,Theta,P] = pcac(fun,Y,V,U,W,Theta,P,params)

sat = @(x) min(max(x,params.pcac_params.u_min),params.pcac_params.u_max);
sat_delta = @(dx) min(max(dx,params.pcac_params.delta_u_min),params.pcac_params.delta_u_max);

guess = [];

x = params.sys_params.x0;
Ts = params.sys_params.Ts;

t = Ts*(0:params.pcac_params.nb_sample-1);

if strcmp(func2str(fun),'pcac_disturbance')

    x_hat = zeros(params.sys_params.n_y*params.rls_params.n_est,1);
    Dist = zeros(params.sys_params.n_y,params.pcac_params.nb_sample);

    for k=1:params.pcac_params.nb_sample-1
    Y(:,k) = ssG.C*x0;

    if k>params.rls_params.n_est
        [u_pcac,x_hat,dist] = fun(k, Y + V, U, Theta(:,k),Dist(:,k), x_hat, params); %PCAC
        U(:,k) = u_pcac;
        Dist(:,k+1) = dist;
    end
    [Theta,P] = rls_code(k, Y + V, U, P, Theta, params); %RLS

    x0 = ssG.A*x0 + ssG.B*(U(:,k)+W(:,k)); %System reponse
%     Y = lsim(params.sys_params.tf,(U+W)',t)'; %System reponse

    end

[Theta,P] = rls_code(k+1, Y + V, U, P, Theta, params); %RLS

else
for k=1:params.pcac_params.nb_sample-1
    Y(:,k) = measure_fct(t(k),x,U(:,k)+W(:,k),params) + V(:,k);

    if k>params.rls_params.n_est
        u_pcac = fun(k, Y + V, U, Theta(:,k), params); %PCAC
        U(:,k+1) = sat(U(:,k) + sat_delta(u_pcac-U(:,k)));
    end

    [Theta,P] = rls_code(k, Y + V, U, P, Theta, params); %RLS
    x = simu_fct(linspace(t(k),t(k+1),2), x,U(:,k)+W(:,k),params); %System reponse
    %Y = lsim(params.sys_params.tf,(U+W)',t)'; %System reponse
end
end

Y(:,k+1) = measure_fct(t(k+1),x,U(:,k+1)+W(:,k+1),params) + V(:,k+1);
[Theta,P] = rls_code(k+1, Y + V, U, P, Theta, params); %RLS

end

