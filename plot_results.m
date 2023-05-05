%% Plot

% Theta
figure(1)
sqrt_nb_var = ceil(sqrt(nb_var));
for k=1:nb_var
subplot(sqrt_nb_var,ceil(nb_var/sqrt_nb_var),k)
%plot(Theta(k,:))
loglog(abs(Theta(k,:) - theta_ref(k))')
xlabel("k (step)")
text = '$|\hat{\theta}_'+string(k)+' - \theta_'+string(k) + '|$';
ylabel(text,'Interpreter','latex','FontWeight','bold')
end
title("Parameter estimation error")

% Input Output
figure(2)
ax1 = subplot(6,1,1);
stairs(t,Y)
hold on
stairs(t,sys_params.ref(t),"--k")
ylabel("Output")
set(gca,'XTickLabel',[])

ax2 = subplot(6,1,2);
stairs(t,U)
hold on
stairs(t,ones(size(t))*pcac_params.u_min,"--k")
stairs(t,ones(size(t))*pcac_params.u_max,"--k")
ylabel("Input")
set(gca,'XTickLabel',[])

ax3 = subplot(6,1,3);
stairs(t,log(abs(sys_params.C_t*Y - sys_params.ref(t))))
ylabel("log|y_k - r_k|")
set(gca,'XTickLabel',[])

% Impulse response and DC
length_impulse = 100;
u_impulse = zeros(sys_params.n_u,length_impulse);
u_impulse(:,1) = 1;
y_impulse = lsim(G,u_impulse',0:length_impulse-1)';
Delta_IR = zeros(1,nb_sample);
Delta_DC = zeros(1,nb_sample);
[num,den] = tfdata(G);
DC = sum(num{1})/sum(den{1});
for k=1:nb_sample
G_hat = tf(Theta(n_est+1:end,k)',[1;Theta(1:n_est,k)]',1);
y_hat_impulse = lsim(G_hat,u_impulse',0:length_impulse-1)';
Delta_IR(k) = norm(y_hat_impulse - y_impulse);
DC_hat = sum(Theta(n_est+1:end,k))/sum(Theta(1:n_est,k));
Delta_DC(k) = abs(DC_hat - DC);
end

ax4 = subplot(6,1,4);
stairs(t,log(Delta_IR))
ylabel("log||\Delta IR||")
set(gca,'XTickLabel',[])

ax5 = subplot(6,1,5);
stairs(t,log(Delta_DC))
xlabel("k (step)")
ylabel("log||\Delta DC||")

linkaxes([ax1,ax2,ax3,ax4,ax5],'x')
% p1 = get(ax1, 'Position');
% p2 = get(ax2, 'Position');
% p3 = get(ax3, 'Position');
% p4 = get(ax4, 'Position');
% p5 = get(ax5, 'Position');
% 
% c = p1(2);
% p1(2) = 1/2*(p1(2) + p2(2) + p2(4));
% p1(4) = p1(4) + c-p1(2);
% p2(4) = (p1(2)-p2(2))*0.8;
% p3(4) = (p2(2)-p3(2))*0.8;
% p4(4) = (p3(2)-p4(2))*0.8;
% p5(4) = (p4(2)-p5(2))*0.8;
% 
% set(ax1, 'pos', p1);%set h1 position
% set(ax2, 'pos', p2);%set h1 position
% set(ax3, 'pos', p3);%set h1 position
% set(ax4, 'pos', p4);%set h1 position
% set(ax5, 'pos', p5);%set h1 position

ax6 = subplot(6,1,6);
% 
% p5 = get(ax5, 'Position');
% p6 = get(ax6, 'Position');
% 
% p6(4) = -(p6(2) - p5(2))*0.7;
% 
% set(ax6, 'pos', p6);%set h1 position

pzmap(G)
hold on
pzmap(G_hat)
legend
