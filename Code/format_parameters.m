function params = format_parameters(params)
% Check consistency of parameters

sys_params = params.sys_params;
rls_params = params.rls_params;
pcac_params = params.pcac_params;

if params.sys_params.sys_type == "LTI"
    % Number of I/O from a LTI system
     sys_params.n_u = size(sys_params.tf.InputDelay,1);
     sys_params.n_y = size(sys_params.tf.OutputDelay,1);
    
    % Theta reference (works only for SISO system)
    [num,den] = tfdata(sys_params.tf);
    num=num{1};den=den{1};
    sys_params.theta_ref = [den(2:end),num]'; 

else
    sys_params.theta_ref = [];
end

% Size of Theta
params.nb_var = rls_params.n_est*sys_params.n_y*sys_params.n_y +... % F_1,...,F_n
                sys_params.n_u*sys_params.n_y +...                  % G_0
                rls_params.n_est*sys_params.n_y*sys_params.n_u;     % G_1,...,G_n

%% Checks

assert(rls_params.n_est>0)

if isfield(rls_params,"lambda")
    assert(rls_params.lambda>0)
else
    assert(rls_params.eta>0)
    assert(rls_params.t_n<rls_params.t_d)
end

assert(all(size(sys_params.C_t) == [size(sys_params.ref(0),1), sys_params.n_y]))

params.sys_params = sys_params;
params.rls_params = rls_params;
params.pcac_params = pcac_params;

end

