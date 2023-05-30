function y = measure_fct(t,x,u,params)
% Measurement function (from state to output)

sys_type = params.sys_params.sys_type;

% Linear
if sys_type == "LTI"
    assert(isfield(params.sys_params,"ss"));
    ss = params.sys_params.ss;
    y = ss.C*x + ss.D*u;

% Nonlinear
else
    assert(isfield(params.sys_params,"g"));
    g = params.sys_params.g;
    y = g(t,x,u);
end

end

