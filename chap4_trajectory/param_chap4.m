clear; close all; clc;
P.gravity = 9.8;
   
%机体物理参数
P.mass = 13.5;
P.Jx   = 0.8244;
P.Jy   = 1.135;
P.Jz   = 1.759; 
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

