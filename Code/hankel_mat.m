function [H_p, H_f] = hankel_mat(data, T_ini, T_f)

[n_x, L, nb_traj] = size(data);

n_c = L - (T_ini + T_f) + 1;

H = zeros(n_x*(T_ini+T_f),n_c*nb_traj);

for j=1:nb_traj
    for i=1:n_c
        H(:,(j-1)*n_c + i) = reshape(data(:,(i-1)+1:i+T_ini+T_f-1,j),[],1);
    end
end

H_p = H(1:n_x*T_ini,:);
H_f = H(n_x*T_ini+1:end,:);

end

