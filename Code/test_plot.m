sys_params = params.sys_params;
rls_params = params.rls_params;
pcac_params = params.pcac_params;

n_y = sys_params.n_y;
n_u = sys_params.n_u;
n_est = rls_params.n_est;
Ts = sys_params.Ts;
%% Plot
linewidth=2;

is_save_fig = isfield(params,'save') && params.save;
is_axes = length(ax_fig2)>0 && length(ax_fig3)>0;

if is_save_fig
    %cd ..\Simulations
    cd .\Simulations
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
    if is_axes
    else
        ax_fig2(1) = subplot(6,1,1);
    end
    stairs(ax_fig2(1),t,(sys_params.C_t*Y)',"LineWidth",linewidth)
    hold on
    if not(is_axes)
        stairs(ax_fig2(1),t,(sys_params.ref(t))',"--k","LineWidth",linewidth)
    end
    ylabel("Output")
    set(gca,'XTickLabel',[])
    title(ax_fig2(1),approach,"FontSize",14)
    legend
    
    if is_axes
    else
        ax_fig2(2) = subplot(6,1,2);
        hold on
    end
    stairs(ax_fig2(2),t,U,"LineWidth",linewidth)
    if not(is_axes)
        stairs(ax_fig2(2),t,ones(size(t))*pcac_params.u_min,"--k","LineWidth",linewidth)
        stairs(ax_fig2(2),t,ones(size(t))*pcac_params.u_max,"--k","LineWidth",linewidth)
    end    
    ylabel("Input")
    set(gca,'XTickLabel',[])
    
    if is_axes
    else
        ax_fig2(3) = subplot(6,1,3);
    end    
    stairs(ax_fig2(3),t,log(abs(sys_params.C_t*Y - sys_params.ref(t)))',"LineWidth",linewidth)
    hold on
    ylabel("log|y_k - r_k|")
    set(gca,'XTickLabel',[])
    
    %Impulse response and DC
%    if params.sys_params.n_y == 1 && params.sys_params.n_u == 1
    length_impulse = 30;
    u_impulse = zeros(sys_params.n_u,length_impulse);
    u_impulse(:,1) = 1;
    y_impulse = lsim(sys_params.tf{1},u_impulse',Ts*(0:length_impulse-1))';
    Delta_IR = zeros(1,pcac_params.nb_sample);
    Delta_DC = zeros(1,pcac_params.nb_sample);

    for k=1:pcac_params.nb_sample
    G_hat = theta_to_tf(Theta(:,k),z,params);
    y_hat_impulse = lsim(G_hat,u_impulse',Ts*(0:length_impulse-1))';
    Delta_IR(k) = norm(y_hat_impulse - y_impulse);
    [num,den] = tfdata(sys_params.tf{1});
    [num_hat,den_hat] = tfdata(G_hat);
    for i=1:n_y
        for j=1:n_u
            DC(i,j)     = sum(num{i,j})/(sum(den{i,j}));
            DC_hat(i,j) = sum(num_hat{i,j})/(sum(den_hat{i,j}));
        end
    end
    Delta_DC(k) = norm(DC_hat - DC);
    end
%     else
%         Delta_IR = exp(1)*ones(size(t));
%         Delta_DC = exp(1)*ones(size(t));
%         G_hat = tf([1],[1],Ts);
%     end

    if is_axes
    else
        ax_fig2(4) = subplot(6,1,4);
    end    
    stairs(ax_fig2(4),t,log(Delta_IR),"LineWidth",linewidth)
    hold on
    ylabel("log||\Delta IR||")
    set(gca,'XTickLabel',[])
    
    if is_axes
    else
        ax_fig2(5) = subplot(6,1,5);
    end    
    stairs(ax_fig2(5),t,log(Delta_DC),"LineWidth",linewidth)
    hold on
    xlabel("k (step)")
    ylabel("log||\Delta DC||")
    

    linkaxes(ax_fig2(1:5),'x')
    p1 = get(ax_fig2(1), 'Position');
    p2 = get(ax_fig2(2), 'Position');
    p3 = get(ax_fig2(3), 'Position');
    p4 = get(ax_fig2(4), 'Position');
    p5 = get(ax_fig2(5), 'Position');
    
    c = p1(2);
    p1(2) = 1/2*(p1(2) + p2(2) + p2(4));
    p1(4) = p1(4) + c-p1(2);
    p2(4) = (p1(2)-p2(2))*0.8;
    p3(4) = (p2(2)-p3(2))*0.8;
    p4(4) = (p3(2)-p4(2))*0.8;
    p5(4) = (p4(2)-p5(2))*0.8;
    
    set(ax_fig2(1), 'pos', p1);%set h1 position
    set(ax_fig2(2), 'pos', p2);%set h1 position
    set(ax_fig2(3), 'pos', p3);%set h1 position
    set(ax_fig2(4), 'pos', p4);%set h1 position
    set(ax_fig2(5), 'pos', p5);%set h1 position
    
    if is_axes
        ax_fig2(6);
    else
        ax_fig2(6) = subplot(6,1,6);
    end    
        
    p5 = get(ax_fig2(5), 'Position');
    p6 = get(ax_fig2(6), 'Position');
    
    p6(4) = -(p6(2) - p5(2))*0.7;
    
    set(ax_fig2(6), 'pos', p6);%set h1 position
    
    pzplot(sys_params.tf{1},G_hat)
    hold on
    legend(["Reference","Estimated"])
    
    a=findobj(gca,'type','line');
    set(a(2),'LineWidth',linewidth);
    set(a(3),'LineWidth',linewidth);
    set(a(4),'LineWidth',linewidth);
    set(a(5),'LineWidth',linewidth);

    % Covariance matrix
    fig3 = figure(3);
    for k=1:pcac_params.nb_sample
    T(k) = trace(P(:,:,k));
    D(k) = det(P(:,:,k));
    end
    
    if is_axes
    ax_fig3(1);
    else
    ax_fig3(1) = subplot(2,1,1);
    end    
    semilogy(ax_fig3(1),T,"LineWidth",linewidth)
    hold on
    xlabel("k (step)")
    text = '$trace(P)$';
    ylabel(text,'Interpreter','latex','FontWeight','bold')
    legend
    
    if is_axes
    ax_fig3(2);
    else
    ax_fig3(2) = subplot(2,1,2);
    end    
    semilogy(ax_fig3(2),D,"LineWidth",linewidth)
    hold on
    xlabel("k (step)")
    text = '$det(P)$';
    ylabel(text,'Interpreter','latex','FontWeight','bold')
    title("Trace and Determinant of the covariance matrix")
    legend
    
    fig4 = figure(4);
    hold on
    plot(Lambda,"LineWidth",linewidth)
    xlabel("k (step)")
    text = '$\lambda$';
    ylabel(text,'Interpreter','latex','FontWeight','bold')
    title("Forgetting coefficient")
    
    fig5 = figure(5);
    plot(Theta(:,1:end-1)',"LineWidth",linewidth)
    hold on
    xlabel("k (step)")
    text = '$\Theta$';
    ylabel(text,'Interpreter','latex','FontWeight','bold')
    title("Estimated coefficients")


else
    fig2 = figure(2);
    if is_axes
    else
        ax_fig2(1) = subplot(3,1,1);
    end
    stairs(ax_fig2(1),t,(sys_params.C_t*Y)',"LineWidth",linewidth)
    hold on
    if not(is_axes)
        stairs(ax_fig2(1),t,sys_params.ref(1:pcac_params.nb_sample)',"--k","LineWidth",linewidth)
    end
    ylabel("Output")
    set(gca,'XTickLabel',[])
    title(ax_fig2(1),approach,"FontSize",14)
    legend
    
    if is_axes
    else
        ax_fig2(2) = subplot(3,1,2);
        hold on
    end
    stairs(ax_fig2(2),t,U',"LineWidth",linewidth)
    if not(is_axes)
        stairs(ax_fig2(2),t,(ones(size(t)).*pcac_params.u_min)',"--k","LineWidth",linewidth)
        stairs(ax_fig2(2),t,(ones(size(t)).*pcac_params.u_max)',"--k","LineWidth",linewidth)
    end    
    ylabel("Input")
    set(gca,'XTickLabel',[])
    
    if is_axes
    else
        ax_fig2(3) = subplot(3,1,3);
    end    
    stairs(ax_fig2(3),t,log(abs(sys_params.C_t*Y - sys_params.ref(1:pcac_params.nb_sample)))',"LineWidth",linewidth)
    hold on
    ylabel("log|y_k - r_k|")
    set(gca,'XTickLabel',[])


    % Covariance matrix
    fig3 = figure(3);
    for k=1:pcac_params.nb_sample
    T(k) = trace(P(:,:,k));
    D(k) = det(P(:,:,k));
    end
    
    if is_axes
    ax_fig3(1);
    else
    ax_fig3(1) = subplot(2,1,1);
    end    
    semilogy(ax_fig3(1),T,"LineWidth",linewidth)
    hold on
    xlabel("k (step)")
    text = '$trace(P)$';
    ylabel(text,'Interpreter','latex','FontWeight','bold')
    legend
    
    if is_axes
    ax_fig3(2);
    else
    ax_fig3(2) = subplot(2,1,2);
    end    
    semilogy(ax_fig3(2),D,"LineWidth",linewidth)
    hold on
    xlabel("k (step)")
    text = '$det(P)$';
    ylabel(text,'Interpreter','latex','FontWeight','bold')
    title("Trace and Determinant of the covariance matrix")
    legend


    fig4 = figure(4);
    hold on
    plot(Lambda,"LineWidth",linewidth)
    xlabel("k (step)")
    text = '$\lambda$';
    ylabel(text,'Interpreter','latex','FontWeight','bold')
    title("Forgetting coefficient")
    
    fig5 = figure(5);
    plot(Theta(:,1:end-1)',"LineWidth",linewidth)
    hold on
    xlabel("k (step)")
    text = '$\Theta$';
    ylabel(text,'Interpreter','latex','FontWeight','bold')
    title("Estimated coefficients")

end

if is_save_fig
    saveas(fig2, "I_0");
    saveas(fig3, "P");
    saveas(fig4, "Lambda");
    saveas(fig5, "Theta");
    save("parameters.mat","params");
    
    cd ..\..\Code\
end
