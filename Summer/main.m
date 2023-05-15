clear; close all;
% profile on
%% Load parameters, model and data
%Example_S_5 % Not working
%Example_S_6 % Not working
Example_1 % Not working
%Example_2 % Not working
%Example_3 % Not working

[Y,U,V,W,Theta,P] = initialize_data(params);

%% PCAC + RLS

[t,Y,U,Theta,P] = pcac(@pcac_rate_based,Y,V,U,W,Theta,P,params);
%[t,Y,U,Theta,P] = pcac(@pcac_normal,Y,V,U,W,Theta,P,params);
%[t,Y,U,Theta,P] = pcac(@pcac_disturbance,Y,V,U,W,Theta,P,params);

plot_results(t,Y,U,Theta,P,params)
%  profile off
%  profile viewer