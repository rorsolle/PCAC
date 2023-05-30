function xnew = simu_fct(tspan,x0,u,params)

sys_type = params.sys_params.sys_type;

if sys_type == "LTI"
    assert(isfield(params.sys_params,"ss"));
    [~,~,xsim] = lsim(params.sys_params.ss,ones(length(tspan),1).*u',tspan-tspan(1),x0);
    xnew = xsim(end,:)';
else
    assert(isfield(params.sys_params,"f"));
    f = params.sys_params.f;
    [~,xsim] = ode45(@(t,y) f(t,y,u), tspan, x0); %System reponse
    xnew = xsim(end,:)';
end

end

