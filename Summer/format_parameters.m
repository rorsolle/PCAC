function params = format_parameters(params)

sys_params = params.sys_params;
rls_params = params.rls_params;
pcac_params = params.pcac_params;


sys_params.n_u = size(sys_params.tf.InputDelay,2);
sys_params.n_y = size(sys_params.tf.OutputDelay,2);
params.nb_var = rls_params.n_est*sys_params.n_y*(sys_params.n_u+sys_params.n_y) +... 
                sys_params.n_u*sys_params.n_y; %Size of theta

[num,den] = tfdata(sys_params.tf);
num=num{1};den=den{1};
sys_params.theta_ref = [den(2:end),num]'; 

assert(rls_params.n_est>0)
assert(rls_params.lambda>0)
assert(rls_params.eta>0)

assert(rls_params.t_n<rls_params.t_d)

assert(all(size(sys_params.C_t) == [size(sys_params.ref(0),1), sys_params.n_y]))

params.sys_params = sys_params;
params.rls_params = rls_params;
params.pcac_params = pcac_params;

end

