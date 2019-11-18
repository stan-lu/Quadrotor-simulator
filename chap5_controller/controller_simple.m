function out = controller_simple(state, des_state, params)
%CONTROLLER  Controller for the quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [x; y; z], state.vel = [x_dot; y_dot; z_dot],
%   state.rot = [phi; theta; psi], state.omega = [p; q; r]
%
%   des_state: The desired states are:
%   des_state.pos = [x; y; z], des_state.vel = [x_dot; y_dot; z_dot],
%   des_state.acc = [x_ddot; y_ddot; z_ddot], des_state.yaw,
%   des_state.yawdot
%
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls


% =================== Your code goes here ===================
state_pos   = state(1:3);
state_vel   = state(4:6);
state_rot   = state(7:9);
state_omega = state(10:12);
des_state_pos = des_state(1:3);
des_state_vel = des_state(4:6);
des_state_acc = des_state(7:9);
des_state_yaw = des_state(10);
des_state_yawdot = des_state(11);


% parameters
k_d = [20; 20; 5];  % position
k_p = [2.8; 2.8; 10];  % position
k_d_att = [1; 1; 1];  % attitude (phi/theta/psi)
k_p_att = [2; 2; 2];  % attitude (phi/theta/psi)

% position controller
r_des_ddot=des_state_acc+...
    k_d.*(des_state_vel-state_vel)+k_p.*(des_state_pos-state_pos);

% attitude controller
phi_des=(r_des_ddot(1)*sin(des_state_yaw)-r_des_ddot(2)*cos(des_state_yaw))...
    /params.gravity;
theta_des=(r_des_ddot(1)*cos(des_state_yaw)+r_des_ddot(2)*sin(des_state_yaw))...
    /params.gravity;
psi_des=des_state_yaw;
p_des=0;
q_des=0;
r_des=des_state_yawdot;

% Thrust
F = params.mass * params .gravity + params.mass * r_des_ddot(3);

% Moment
M=zeros(3,1);
M(1)=k_p_att(1)*(phi_des-state_rot(1)) +k_d_att(1)*(p_des-state_omega(1));
M(2)=k_p_att(2)*(theta_des-state_rot(2))+k_d_att(2)*(q_des-state_omega(2));
M(3)=k_p_att(3)*(psi_des-state_rot(3))  +k_d_att(3)*(r_des-state_omega(3));

out = [F;M;des_state_vel;phi_des;theta_des;psi_des;p_des;q_des;r_des];

% =================== Your code ends here ===================

end
