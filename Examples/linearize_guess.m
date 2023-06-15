function theta = linearize_guess(x0,sys_params,rls_params)

n_est = rls_params.n_est;
n_y = sys_params.n_y;
n_u = sys_params.n_u;
n_x = length(x0);
Ts = sys_params.Ts;

t=0;

syms x [n_x,1]
syms u [n_u,1]
f_syms_x = sys_params.f(t,x,zeros(n_u,1));
f_syms_u = sys_params.f(t,x0,u);
g_syms_x = sys_params.g(t,x,zeros(n_u,1));
g_syms_u = sys_params.g(t,x0,u);

J_f_syms_x = jacobian(f_syms_x,x);
J_f_syms_u = jacobian(f_syms_u,u);
J_g_syms_x = jacobian(g_syms_x,x);
J_g_syms_u = jacobian(g_syms_u,u);
J_f_x=subs(J_f_syms_x,x,x0);
J_f_u=subs(J_f_syms_u,u,zeros(n_u,1));
J_g_x=subs(J_g_syms_x,x,x0);
J_g_u=subs(J_g_syms_u,u,zeros(n_u,1));

G = ss(double(J_f_x),double(J_f_u),double(J_g_x),double(J_g_u));
Gd = c2d(G,Ts);
A = Gd.A;
B = Gd.B;
C = Gd.C;
D = Gd.D;

Obs_mat_x = zeros(n_x*n_y,n_x);
Obs_mat_u = kron(eye(n_x),D);
Con_mat = zeros(n_x,n_u*n_x);

for i=1:n_x
    Obs_mat_x((i-1)*n_y+1:i*n_y,:) = C*A^(i-1);
    for j=1:i-1
        Obs_mat_u((i-1)*n_y+1:i*n_y,(j-1)*n_u+1:j*n_u) = C*A^(i-j-1)*B;
    end
    Con_mat(1:n_x,(i-1)*n_u+1:i*n_u) = A^(n_x-i)*B;
end

F_mat = C*A^n_x*pinv(Obs_mat_x);
G_mat = C*(Con_mat - A^n_x*pinv(Obs_mat_x)*Obs_mat_u);
F_mat_1 = zeros(size(F_mat));
G_mat_1 = zeros(size(G_mat));
G_0 = D;

for i=1:n_x
    F_mat_1(1:n_y,(i-1)*n_y+1:i*n_y) = F_mat(1:n_y,(n_x-i)*n_y+1:(n_x-(i-1))*n_y);
    G_mat_1(1:n_y,(i-1)*n_u+1:i*n_u) = G_mat(1:n_y,(n_x-i)*n_u+1:(n_x-(i-1))*n_u);
end

if n_est <= n_x
    theta = [-reshape(F_mat_1(1:n_y,1:n_y*n_est),1,[]),reshape(G_0,1,[]),reshape(G_mat_1(1:n_y,1:n_u*n_est),1,[])]';
else
    theta = [-reshape([F_mat_1,zeros(n_y,(n_est-n_x)*n_y)],1,[]),reshape(G_0,1,[]),reshape([G_mat_1,zeros(n_y,(n_est-n_x)*n_u)],1,[])]';
end

end

