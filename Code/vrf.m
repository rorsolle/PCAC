function g = vrf(Z, params)

rls_params = params.rls_params;
t_d = rls_params.t_d;
t_n = rls_params.t_n;

assert(t_d>= t_n)
assert(size(Z,2) == t_d+1)

if size(Z,1)>1
g = sqrt(mean(diag(Z(:,t_d-t_n+1:end)'*Z(:,t_d-t_n+1:end))))/...
    sqrt(mean(diag(Z'*Z))) - 1;
else
    g = sqrt(mean(Z(t_d-t_n+1:end)*Z(t_d-t_n+1:end)'))/...
    sqrt(mean(Z*Z')) - 1;
end

end

