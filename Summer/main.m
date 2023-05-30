clear; %close all;
% profile on

%% Load parameters, model and data
%Example_S_5 % Working
%Example_S_6 % Working
Example_1 % Working
%Example_2 % Not working
%Example_3 % Not working
%Example_single_pendulum % Not working
%Example_double_pendulum % Not working
%Example_deep_stall % Not working

[Y,U,V,W,Theta,P] = initialize_data(params);

%% PCAC + RL
tic

% Choose your algorithm

%fun = @pcac_paper;
fun = @pcac_rate_based;

% PCAC
[t,Y,U,Theta,P] = pcac(fun,Y,V,U,W,Theta,P,params);

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
end

% Plots
plot_results(t,Y,U,Theta,P,params,approach)

% profile off
% profile viewer
