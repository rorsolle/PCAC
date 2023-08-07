function g = vrf(Z, params)

rls_params = params.rls_params;
t_d = rls_params.t_d;
t_n = rls_params.t_n;
p = params.sys_params.n_y;
alpha = rls_params.alpha;

assert(t_d>= t_n)
assert(size(Z,2) == t_d+1)

if size(Z,1)>1
g = sqrt(mean(diag(Z(:,t_d-t_n+1:end)'*Z(:,t_d-t_n+1:end))))/...
    sqrt(mean(diag(Z'*Z))) - 1;
else
    g = sqrt(1/t_n*(Z(t_d-t_n+1:end)*Z(t_d-t_n+1:end)'))/...
    sqrt(1/t_d*(Z*Z')) - 1;
end

% sdtemp = Z - mean(Z,2);
% sd = sdtemp*sdtemp'/t_d;
% 
% sntemp = Z(:,t_d-t_n+1:end) - mean(Z(:,t_d-t_n+1:end),2);
% sn = sntemp*sntemp'/t_n;
% 
% if (det(sd)^(1/p) < 1e-10) || Z(end) == 0; 100*eps; %( sd == 0 )
%     g = 0;
% else
%     %Lawley - Hotelling Trace
%     h = t_n; e = t_d; %dof
%     a = p*h;
%     B = ((e+h-p-1)*(e-1))/((e-p-3)*(e-p));
%     b = 4+(a+2)/(B-1);
%     c = a*(b-2)/(b*(e-p-1));
%     H = (t_n)*sn; %sum of squares formulation of variance
%     E = (t_d)*sd; %sum of squares
%     T = trace(H*E^-1); 
%     F = T/c;
%     
%     testvar = F;
% 
%     if testvar < 1 %F-Test, %only care about if the numerator is larger?
%         g  = -10; %sd/sn - finv(0.999999,obj.tau_d,obj.tau_n);
%     else
%         g  = sqrt(testvar) - sqrt(finv(1-alpha,a,b)); %variance was computed with obj.tau_n/d + 1 elements
%     end
%     if g > 0
%         %gval = sqrt(gval);
%     end
% end

end

