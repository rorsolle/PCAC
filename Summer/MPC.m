function u = MPC(A,B,C,x0,A_ineq_y,B_ineq_y,A_ineq_u,B_ineq_u,lb_x,ub_x,lb_u,ub_u,Q,P,R,N,is_ordered_input,is_diff_input)
% min x^TQx + u^TRu st x(k+1) = Ax + Bu, y(k) = Cx, A_ineq*y<= B_ineq
% N : horizon
% X = [x0,...,xN,u0,...,uN-1]

nx = size(A,1);
nu = size(B,2);
ny = size(C,1);

Aeq_1 = kron(eye(N+1), eye(nx)) + kron(diag(ones(N,1), -1), -A);
Aeq_inter = kron(diag(ones(N,1), -1), -B);

Aeq_2 = zeros((N+1)*nx,N*nu) + Aeq_inter(:,1:end-nu);

Aeq = [Aeq_1,Aeq_2];

Beq = zeros(nx*(N+1),1);
Beq(1:nx) = x0;

if (is_ordered_input)
% Input : U_k = [u(k),u(k-1),...,u(k-n)]
% u_k(i) = u_(k+1)(i-1) 
    Aeq_u_1 = kron(eye(N),eye(nu-1,nu));
    B = diag(-1*ones(nu-1,1),1);
    B = B(1:end-1,:);
    Aeq_u_2 = kron(diag(1*ones(N-1,1),1),B);
    Aeq_u = Aeq_u_1(1:end-(nu-1),:) + Aeq_u_2(1:end-(nu-1),:);
    
    Aeq = [Aeq;
           zeros((N-1)*(nu-1),(N+1)*nx),Aeq_u];
    
    Beq = [Beq;
           zeros((N-1)*(nu-1),1)];
end

if (is_diff_input)
% Delta_u_0 (k-i) = U_0(k-i) - U_0(k-i-1)    
    Aeq = [Aeq;
       zeros((nu-1),(N+1)*nx+1),eye(nu-1),zeros(nu-1,(N-1)*nu)];
    Beq = [Beq;
        -diff(x0(end-nu+1:end))];
end



A_ineq = blkdiag(kron(eye(N+1),A_ineq_y*C),kron(eye(N),A_ineq_u));
B_ineq = [kron(ones(N+1,1),B_ineq_y);kron(ones(N,1),B_ineq_u)];

LB = [kron(ones(N+1,1),lb_x);
      kron(ones(N,1),lb_u)];

UB = [kron(ones(N+1,1),ub_x);
      kron(ones(N,1),ub_u)];

H = blkdiag(kron(eye(N),Q),P,kron(eye(N),R));

opts = optimset("Display","None");
MPC = quadprog(H,zeros((N+1)*nx+N*nu,1),A_ineq,B_ineq,Aeq,Beq,LB,UB,[],opts);
if isempty(MPC)
    u = zeros(nu,1);
%     next_guess = [];
else
    u = MPC((N+1)*nx+1:(N+1)*nx+nu);
%     idx = [nx + (1:N*nx),(N*nx+1:(N+1)*nx),(N+1)*nx + nu + (1:(N-1)*nu),((N+1)*nx+(N-1)*nu+1:(N+1)*nx+N*nu)];
%     next_guess = MPC(idx);
end

end