clear; close all;
% profile on

%% Load parameters, model and data

%cd .\Examples

Example_1 
%Example_2
%Example_3
%Example_4
%Example_5
%Switch_single_pendulum
%Example_switch_system
%Example_switch_system_v2
%Example_single_pendulum
%Example_double_pendulum
%Example_duffing
%Example_ball_and_beam
%Example_three_body
%Example_deep_stall
 
%cd ../Code

params = format_parameters(params); % Format parameters

ax_fig2 = [];
ax_fig3 = [];
% switch_idx = [50,100,150];
% for k=1:3
%  %   params.rls_params = rmfield(params.rls_params,'lambda');
%      %params.pcac_params.R = 1/10^10;
%      params.sys_params.switch = [switch_idx(k)];
% if k>2
%     params.save=1;
% end
[Y,U,V,W,Theta,P,Lambda] = initialize_data(params);

%% PCAC + RL
tic

% Choose your algorithm
%fun = @pcac_paper;
%fun = @pcac_paper_test;
%fun = @pcac_rate_based;
fun = @deepc;

% PCAC
%[t,Y,U,Theta,P,Lambda] = pcac(fun,Y,V,U,W,Theta,P,Lambda,params);

% DeePC
[t,Y,U,data_Y,data_U] = deepc_run(Y,V,U,W,params);

% ODeePC
%[t,Y,U] = odeepc_run(Y,V,U,W,params);

toc

switch func2str(fun)
    case "pcac_normal"
        approach = "Normal Approach";
    case "pcac_disturbance"
        approach = "Disturbance Approach";
    case "pcac_paper"
        approach = "Paper Approach";
    case "pcac_paper_test"
        approach = "Paper Test Approach";
    case "pcac_rate_based"
        approach = "Rate-based Approach";
    case "deepc"
        approach = "DeePC Approach";
end
% Plots
%plot_results(t,Y,U,Theta,P,Lambda,params,approach)
%test_plot
test_plot_deepc
%plot_results_deepc(t,Y,U,params,approach)

% profile off
% profile viewer
 
%cd ../