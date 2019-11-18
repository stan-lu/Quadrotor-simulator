function params=att_params()
% 姿态控制相关的参数

% params.att_p=[6.5;6.5;2.8];
% 
% params.rate_p=[0.15;0.15;0.2];
% params.rate_i=[0.05;0.05;0.1];
% params.rate_d=[0.003;0.003;0];
% params.rate_ff=[0;0;0];

params.att_p=[1.2; 1.2; 0.2];

params.rate_p=[2; 2; 0];
params.rate_i=[0; 0; 0];
params.rate_d=[0.6; 0.6; 0];
params.rate_ff=[0; 0; 0];

params.ratesMax = [120*pi/180; 120*pi/180; 45*pi/180];

params.tpa_slope=0;
params.tpa_breakpoint=1;

params.yaw_ff=0.5;

end