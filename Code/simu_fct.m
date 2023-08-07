function xnew = simu_fct(tspan,x0,u,params)

sys_type = params.sys_params.sys_type;

if sys_type == "LTI"
    assert(isfield(params.sys_params,"ss"));
    assert(isfield(params.sys_params,"switch"));
    first_idx = find(params.sys_params.switch>tspan(1),1);
    if isempty(first_idx)
        idx_sys = length(params.sys_params.switch)+1;
    else
        idx_sys = first_idx;
    end
    [~,~,xsim] = lsim(params.sys_params.ss{idx_sys},ones(length(tspan),1).*u',tspan-tspan(1),x0);

    xnew = xsim(end,:)';
else
    assert(isfield(params.sys_params,"f"));
    f = params.sys_params.f;
    opts = odeset('RelTol',1e-12,'AbsTol',1e-12);
    [~,xsim] = ode23(@(t,y) f(t,y,u), tspan, x0, opts); %System reponse
    xnew = xsim(end,:)';
end

end

