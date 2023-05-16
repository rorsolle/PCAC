clear; close all;
% profile on

%% Load parameters, model and data
%Example_S_5 % Working
%Example_S_6 % Working
Example_1 % Working
%Example_2 % Not working
%Example_3 % Not working

[Y,U,V,W,Theta,P] = initialize_data(params);

%% PCAC + RLS
tic

[t,Y,U,Theta,P] = pcac(@pcac_rate_based,Y,V,U,W,Theta,P,params);
%[t,Y,U,Theta,P] = pcac(@pcac_normal,Y,V,U,W,Theta,P,params);
%[t,Y,U,Theta,P] = pcac(@pcac_disturbance,Y,V,U,W,Theta,P,params);
%[t,Y,U,Theta,P] = pcac(@pcac_paper,Y,V,U,W,Theta,P,params);

toc
plot_results(t,Y,U,Theta,P,params)
%  profile off
%  profile viewer
