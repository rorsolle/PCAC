function y = measure_fct(t,x,u,params)
% Measurement function (from state to output)

sys_type = params.sys_params.sys_type;

% Linear
if sys_type == "LTI"
    assert(isfield(params.sys_params,"ss"));
    assert(isfield(params.sys_params,"switch"));

    first_idx = find(params.sys_params.switch>t(1),1);
    if isempty(first_idx)
        idx_sys = length(params.sys_params.switch)+1;
    else
        idx_sys = first_idx;
    end
    ss = params.sys_params.ss{idx_sys};
    y = ss.C*x + ss.D*u;

% Nonlinear
else
    assert(isfield(params.sys_params,"g"));
    g = params.sys_params.g;
    y = g(t,x,u);
end

end

