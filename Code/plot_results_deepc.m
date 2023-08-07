function [figs] = plot_results_deepc(t,Y,U,params,approach)
sys_params = params.sys_params;
rls_params = params.rls_params;
pcac_params = params.pcac_params;

n_est = rls_params.n_est;

%% Plot
linewidth=2;

if isfield(params,'save') && params.save
    cd ..\Simulations
    new_sim = "Sim"+string(datetime)+".mat";
    new_sim=strrep(new_sim,' ','_');
    new_sim=strrep(new_sim,'-','_');
    new_sim=strrep(new_sim,':','_');
    status = mkdir(new_sim);
    [~] = cd(new_sim);
end

% Input Output
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

if isfield(params,'save') && params.save
    saveas(fig2, "I_0");
    save("parameters.mat","params");
    
    cd ..\..\Code\
end

end
