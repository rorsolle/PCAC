clear; %close all;
% profile on

%% Load parameters, model and data

cd .\Examples

%Example_S_5
%Example_S_6
%Example_1 
%Example_2
%Example_3
Example_single_pendulum
%Example_double_pendulum
%Example_deep_stall
 
cd ../Code

params = format_parameters(params); % Format parameters

[Y,U,V,W,Theta,P] = initialize_data(params);

%% PCAC + RL
tic

% Choose your algorithm

fun = @pcac_paper;
%fun = @pcac_rate_based;

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

cd ../
