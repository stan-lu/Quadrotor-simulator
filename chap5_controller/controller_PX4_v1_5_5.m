function out = controller_PX4_v1_5_5(state, des_state, t, params)
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
% des_state_vel = des_state(4:6);
% des_state_acc = des_state(7:9);
des_state_yaw = des_state(10);
des_state_yawdot = des_state(11);



%     pos_sp = [des_state_pos(1); des_state_pos(2); des_state_pos(3)];
%     pos_sp = [1; 1; 1];
vel_sp = Position_Controller(des_state_pos, state_pos, params);

q_curr = euler2q(state_rot);
% [thrust_sp, att_sp, att_sp_thrust] = ...
%     Velocity_Controller(vel_sp, state.vel, t, q_curr, des_state.yaw);
[thrust_sp, att_sp, att_sp_thrust] = ...
    Velocity_Controller(vel_sp, state_vel, q_curr, des_state_yaw, t, params);


rates_sp = Attitude_Controller_v1_5_5(att_sp, q_curr, des_state_yawdot, params);
% if t ~= 0
%     att_control = ...
%         Attitude_Rates_Controller(rates_sp, state.omega,0.01, 0);
% else
att_control = ...
    Attitude_Rates_Controller(rates_sp, state_omega, t, params);
% end

F = params.mass * params.gravity + params.mass * att_sp_thrust;
% F = params.mass * params.gravity;
% M = [att_control(1); 0; 0];
I = [params.Jx,         0,     params.Jxz;
             0, params.Jy,              0;
    params.Jxz,         0,      params.Jz
    ];

M = I*att_control;
% M = zeros(3, 1);
% M = [0; 0; 0.00001];

out = [F;M;vel_sp;q2euler(att_sp);rates_sp];


% =================== Your code ends here ===================

end
