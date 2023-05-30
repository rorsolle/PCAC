function [x_hat_new,d_new] = obs_dist(x_hat,u,d,y,A,B,B_d,C)

A_tilde = [A,B_d;zeros(length(d),length(A)),eye(length(d))];
C_tilde = [C,zeros(size(C,1),length(d))];

noise = 10000;
[M,~,~,~] = dlqe(A_tilde,noise*eye(size(A_tilde)),C_tilde,noise*eye(size(A_tilde)),noise*eye(size(C_tilde,1)));

L1 = A*M(1:length(A),:) + B_d*M(length(A)+1:end,:);
L2 = M(length(A)+1:end,:);

x_hat_new = A*x_hat + B*u + B_d*d + L1*(y - C*x_hat);
d_new = d + L2*(y - C*x_hat);
L2
end

