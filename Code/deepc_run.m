function [t,Y,U,data_Y,data_U] = deepc_run(Y,V,U,W,params)

sat = @(x) min(max(x,params.pcac_params.u_min),params.pcac_params.u_max);
sat_delta = @(dx) min(max(dx,params.pcac_params.delta_u_min),params.pcac_params.delta_u_max);

Ts = params.sys_params.Ts;

% data_Y = zeros(params.sys_params.n_y,params.deepc_params.T_d,params.deepc_params.nb_try);
% data_U = (params.pcac_params.u_max - params.pcac_params.u_min).*rand(params.sys_params.n_u,params.deepc_params.T_d,params.deepc_params.nb_try) + params.pcac_params.u_min;
% data_V = params.sys_params.std_v.*randn(params.sys_params.n_y,params.deepc_params.T_d,params.deepc_params.nb_try);
% data_W = params.sys_params.std_w.*randn(params.sys_params.n_u,params.deepc_params.T_d,params.deepc_params.nb_try);
% 
% 
% t = Ts*(0:params.deepc_params.T_d-1);
% 
% for ii=1:params.deepc_params.nb_try
% x = params.sys_params.x0;
% for k=1:params.deepc_params.T_d-1
%     data_Y(:,k,ii) = measure_fct(t(k),x,data_U(:,k,ii) + data_W(:,k,ii),params) + data_V(:,k,ii);
%     x = simu_fct(linspace(t(k),t(k+1),2), x, data_U(:,k,ii)+data_W(:,k,ii),params); %System reponse
% end
% data_Y(:,k+1,ii) = measure_fct(t(k+1),x,data_U(:,k+1,ii)+data_W(:,k+1,ii),params) + data_V(:,k+1,ii);
% end

t = Ts*(0:params.pcac_params.nb_sample-1);
x = params.sys_params.x0;

%data_Y = zeros(params.sys_params.n_y, params.pcac_params.nb_sample,1);
%data_U = zeros(params.sys_params.n_u, params.pcac_params.nb_sample,1);

old_g = 0;

for k=1:params.pcac_params.nb_sample-1
    Y(:,k) = measure_fct(t(k),x,U(:,k)+W(:,k),params) + V(:,k);
    
    if k<=params.deepc_params.max_data
        data_Y(:,k,1) = Y(:,k);
        data_U(:,k,1) = U(:,k);
    else
        data_Y = [data_Y(:,2:end,1),Y(:,k)];
        data_U = [data_U(:,2:end,1),U(:,k)];
    end

    if k>min(params.deepc_params.T_ini,params.deepc_params.max_data)+1
        [u_deepc,g] = deepc(k, Y + V, U, data_Y, data_U, old_g, params); %PCAC
        old_g = g*0;
        %U(:,k+1) = sat(u_deepc);
        U(:,k+1) = sat(U(:,k) + sat_delta(u_deepc-U(:,k)));
     else
        U(:,k+1) = (params.pcac_params.u_max + params.pcac_params.u_min)/2 + 0.3*(params.pcac_params.u_max - params.pcac_params.u_min)/2.*(2*rand(params.sys_params.n_u,1)-1);
    end
    x = simu_fct(linspace(t(k),t(k+1),2), x,U(:,k)+W(:,k),params); %System reponse
end
Y(:,k+1) = measure_fct(t(k+1),x,U(:,k+1)+W(:,k+1),params) + V(:,k+1);

end

