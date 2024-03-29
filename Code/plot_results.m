function [] = plot_results(t,Y,U,Theta,P,Lambda,params,approach)
sys_params = params.sys_params;
rls_params = params.rls_params;
pcac_params = params.pcac_params;

n_est = rls_params.n_est;

%% Plot
linewidth=2;

is_save_fig = isfield(params,'save') && params.save;
if is_save_fig
    cd ..\Simulations
    new_sim = "Sim"+string(datetime)+".mat";
    new_sim=strrep(new_sim,' ','_');
    new_sim=strrep(new_sim,'-','_');
    new_sim=strrep(new_sim,':','_');
    status = mkdir(new_sim);
    [~] = cd(new_sim);
end

figs = [];

% Theta
if params.nb_var == length(sys_params.theta_ref)
    fig1 = figure(1);
    sqrt_nb_var = ceil(sqrt(params.nb_var));
    for k=1:params.nb_var
        hold on
    subplot(sqrt_nb_var,ceil(params.nb_var/sqrt_nb_var),k)
    loglog(abs(Theta(k,:) - sys_params.theta_ref(k))',"LineWidth",linewidth)
    xlabel("k (step)")
    text = '$|\hat{\theta}_'+string(k)+' - \theta_'+string(k) + '|$';
    ylabel(text,'Interpreter','latex','FontWeight','bold')
    end
    title("Parameter estimation error")
    
    if params.save
        saveas(fig1, "Theta_err");
    end
    figs = [figs,fig1];

end


% Input Output

if sys_params.sys_type == "LTI"
    fig2 = figure(2);
    ax1 = subplot(6,1,1);
    stairs(t,(sys_params.C_t*Y)',"LineWidth",linewidth)
    hold on
    stairs(t,sys_params.ref(t),"--k","LineWidth",linewidth)
    ylabel("Output")
    set(gca,'XTickLabel',[])
    title(approach,"FontSize",14)
    
    ax2 = subplot(6,1,2);
    stairs(t,U,"LineWidth",linewidth)
    hold on
    stairs(t,ones(size(t))*pcac_params.u_min,"--k","LineWidth",linewidth)
    stairs(t,ones(size(t))*pcac_params.u_max,"--k","LineWidth",linewidth)
    ylabel("Input")
    set(gca,'XTickLabel',[])
    
    ax3 = subplot(6,1,3);
    stairs(t,log(abs(sys_params.C_t*Y - sys_params.ref(t))),"LineWidth",linewidth)
    ylabel("log|y_k - r_k|")
    set(gca,'XTickLabel',[])
    
    %Impulse response and DC
    length_impulse = 30;
    u_impulse = zeros(sys_params.n_u,length_impulse);
    u_impulse(:,1) = 1;
    y_impulse = lsim(sys_params.tf{1},u_impulse',0:length_impulse-1)';
    Delta_IR = zeros(1,pcac_params.nb_sample);
    Delta_DC = zeros(1,pcac_params.nb_sample);
    [num,den] = tfdata(sys_params.tf{1});
    DC = sum(num{1})/(sum(den{1})-1);
    for k=1:pcac_params.nb_sample
    G_hat = tf(Theta(n_est+1:end,k)',[1;Theta(1:n_est,k)]',1);
    y_hat_impulse = lsim(G_hat,u_impulse',0:length_impulse-1)';
    Delta_IR(k) = norm(y_hat_impulse - y_impulse);
    DC_hat = sum(Theta(n_est+1:end,k))/sum(Theta(1:n_est,k));
    Delta_DC(k) = abs(DC_hat - DC);
    end
    
    ax4 = subplot(6,1,4);
    stairs(t,log(Delta_IR),"LineWidth",linewidth)
    hold on
    ylabel("log||\Delta IR||")
    set(gca,'XTickLabel',[])
    
    ax5 = subplot(6,1,5);
    stairs(t,log(Delta_DC),"LineWidth",linewidth)
    hold on
    xlabel("k (step)")
    ylabel("log||\Delta DC||")
    
    linkaxes([ax1,ax2,ax3,ax4,ax5],'x')
    p1 = get(ax1, 'Position');
    p2 = get(ax2, 'Position');
    p3 = get(ax3, 'Position');
    p4 = get(ax4, 'Position');
    p5 = get(ax5, 'Position');
    
    c = p1(2);
    p1(2) = 1/2*(p1(2) + p2(2) + p2(4));
    p1(4) = p1(4) + c-p1(2);
    p2(4) = (p1(2)-p2(2))*0.8;
    p3(4) = (p2(2)-p3(2))*0.8;
    p4(4) = (p3(2)-p4(2))*0.8;
    p5(4) = (p4(2)-p5(2))*0.8;
    
    set(ax1, 'pos', p1);%set h1 position
    set(ax2, 'pos', p2);%set h1 position
    set(ax3, 'pos', p3);%set h1 position
    set(ax4, 'pos', p4);%set h1 position
    set(ax5, 'pos', p5);%set h1 position
    
    ax6 = subplot(6,1,6);
    
    p5 = get(ax5, 'Position');
    p6 = get(ax6, 'Position');
    
    p6(4) = -(p6(2) - p5(2))*0.7;
    
    set(ax6, 'pos', p6);%set h1 position
    
    pzplot(sys_params.tf{1},G_hat)
    hold on
    legend(["Reference","Estimated"])
    
    a=findobj(gca,'type','line');
    set(a(2),'LineWidth',linewidth);
    set(a(3),'LineWidth',linewidth);
    set(a(4),'LineWidth',linewidth);
    set(a(5),'LineWidth',linewidth);

else
    fig2 = figure(2);
    ax1 = subplot(3,1,1);
    stairs(t,(sys_params.C_t*Y)',"LineWidth",linewidth)
    hold on
    stairs(t,sys_params.ref(1:length(t))',"--k","LineWidth",linewidth)
    ylabel("Output")
    set(gca,'XTickLabel',[])
    title(approach,"FontSize",14)
    
    ax2 = subplot(3,1,2);
    stairs(t,U',"LineWidth",linewidth)
    hold on
%    stairs(t,ones(size(t))*pcac_params.u_min,"--k","LineWidth",linewidth)
%    stairs(t,ones(size(t))*pcac_params.u_max,"--k","LineWidth",linewidth)
    ylabel("Input")
    set(gca,'XTickLabel',[])
    
    ax3 = subplot(3,1,3);
    stairs(t,log(abs(sys_params.C_t*Y - sys_params.ref(1:length(t))))',"LineWidth",linewidth)
    xlabel("Time (seconds)")
    ylabel("log|y_k - r_k|")

    linkaxes([ax1,ax2,ax3],'x')
end

% Covariance matrix
fig3 = figure(3);
for k=1:pcac_params.nb_sample
    T(k) = trace(P(:,:,k));
    D(k) = det(P(:,:,k));
end

subplot(2,1,1)
semilogy(T,"LineWidth",linewidth)
xlabel("k (step)")
text = '$trace(P)$';
ylabel(text,'Interpreter','latex','FontWeight','bold')
subplot(2,1,2)
semilogy(D,"LineWidth",linewidth)
xlabel("k (step)")
text = '$det(P)$';
ylabel(text,'Interpreter','latex','FontWeight','bold')
title("Trace and Determinant of the covariance matrix")

fig4 = figure(4);
hold on
plot(Lambda,"LineWidth",linewidth)
xlabel("k (step)")
text = '$\lambda$';
ylabel(text,'Interpreter','latex','FontWeight','bold')
title("Forgetting coefficient")

fig5 = figure(5);
plot(Theta(:,1:end-1)',"LineWidth",linewidth)
xlabel("k (step)")
text = '$\Theta$';
ylabel(text,'Interpreter','latex','FontWeight','bold')
title("Estimated coefficients")

if is_save_fig
    saveas(fig2, "I_0");
    saveas(fig3, "P");
    saveas(fig4, "Lambda");
    saveas(fig5, "Theta");
    save("parameters.mat","params");
    
    cd ..\..\Code\
end

figs = [figs,fig2,fig3,fig4,fig5];

end
