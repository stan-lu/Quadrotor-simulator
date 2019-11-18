clear; close all; clc;
P.gravity = 9.81;
   
%机体物理参数
P.mass = 0.81;
P.Jx   = 0.00025;
P.Jy   = 0.00025;
P.Jz   = 0.0003738; 
P.Jxz  = 0;

% 初始空速
P.Va0 = 0;
gamma = 0; 
% initial conditions初始条件
P.px0    = 0;   % initial x position
P.py0    = 0;   % initial y position
P.pz0    = 0;   % initial z position
P.u0     = P.Va0;  % initial velocity along body x-axis
P.v0     = 0;  % initial velocity along body y-axis
P.w0     = 0;% initial velocity along body z-axis
P.phi0   = 0;  % initial roll angle
P.theta0 = 0;  % initial pitch angle
P.psi0   = 0;  % initial yaw angle
P.p0     = 0;  % initial body frame roll rate
P.q0     = 0;  % initial body frame pitch rate
P.r0     = 0;  % initial body frame yaw rate

P.Ts = 0.01;

P.pos_p = [15; 15; 10];

% params.vel_p = [0.05; 0.05; 3];
P.vel_p = [35; 35; 100];
P.vel_i = [0; 0; 0];
% params.vel_d = [0.0005; 0.0005; 0.015];
P.vel_d = [0.0001; 0.0001; 0.015];


P.att_p=[30; 30; 15];

% P.rate_p=[0.06; 0.06; 0.1];
P.rate_p=[120; 120; 70];
P.rate_i=[0; 0; 0];
P.rate_d=[0; 0; 0];
P.rate_ff=[0.001; 0.001; 0];

P.ratesMax = [120*pi/180; 120*pi/180; 45*pi/180];

P.tpa_slope = 0;
P.tpa_breakpoint = 1;

P.yaw_ff = 0.5;


P.theta_x   = 70 *pi/180;
P.theta_y   = 70 *pi/180;