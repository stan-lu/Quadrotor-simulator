function params=pos_params()

% params.pos_p=[0.95;0.95;1];
% params.vel_p=[0.09;0.09;0.2];
% params.vel_i=[0.02;0.02;0.02];
% params.vel_d=[0.01;0.01;0];

params.Ts = 0.01;

params.pos_p = [40; 40; 2];

params.vel_p = [120; 120; 0.8];
params.vel_i = [0; 0; 0];
params.vel_d = [40; 40; 0];

params.thr_hover = 0.5;  % 悬停时的油门量

end