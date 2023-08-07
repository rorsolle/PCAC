sys_params = params.sys_params;
rls_params = params.rls_params;
pcac_params = params.pcac_params;

n_est = rls_params.n_est;

%% Plot
linewidth=2;

is_save_fig = isfield(params,'save') && params.save;
is_axes = length(ax_fig2)>0;

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

% Input Output

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
    
if is_save_fig
    saveas(fig2, "I_0");
    save("parameters.mat","params");
    
    cd ..\..\Code\
end
