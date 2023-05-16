function [t,Y,U,Theta,P] = pcac(fun,Y,V,U,W,Theta,P,params)

sat = @(x) min(max(x,params.pcac_params.u_min),params.pcac_params.u_max);
t = 0:params.pcac_params.nb_sample-1;

guess = [];

ssG = ss(params.sys_params.tf);
x0 = zeros(size(ssG.A,1),1);

if strcmp(func2str(fun),'pcac_disturbance')

    x_hat = zeros(params.sys_params.n_y*params.rls_params.n_est,1);
    Dist = zeros(params.sys_params.n_y,params.pcac_params.nb_sample);

    for k=1:params.pcac_params.nb_sample-1
    Y(:,k) = ssG.C*x0;
   
    if k>params.rls_params.n_est
        [u_pcac,x_hat,dist] = fun(k, Y + V, U, Theta(:,k),Dist(:,k), x_hat, params); %PCAC
        U(:,k) = sat(u_pcac);
        Dist(:,k+1) = dist;
    end
    [Theta,P] = rls_code(k, Y + V, U, P, Theta, params); %RLS

    x0 = ssG.A*x0 + ssG.B*(U(:,k)+W(:,k)); %System reponse
%     Y = lsim(params.sys_params.tf,(U+W)',t)'; %System reponse

    end

[Theta,P] = rls_code(k+1, Y + V, U, P, Theta, params); %RLS

else

for k=1:params.pcac_params.nb_sample-1
    Y(:,k) = ssG.C*x0;
    if k>params.rls_params.n_est
        u_pcac = fun(k, Y + V, U, Theta(:,k), params); %PCAC
        U(:,k) = sat(u_pcac);
    end

    [Theta,P] = rls_code(k, Y + V, U, P, Theta, params); %RLS

    x0 = ssG.A*x0 + ssG.B*(U(:,k)+W(:,k)); %System reponse
    %Y = lsim(params.sys_params.tf,(U+W)',t)'; %System reponse
end
end

Y(:,k+1) = ssG.C*x0;
[Theta,P] = rls_code(k+1, Y + V, U, P, Theta, params); %RLS

end

