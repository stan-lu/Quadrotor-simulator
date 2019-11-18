function [thrust_sp, att_sp, att_sp_thrust] = ...
    Velocity_Controller(vel_sp, vel_curr, q_curr, yaw, t, para)

persistent vel_prev
persistent vel_int

if t==0
    vel_prev = vel_curr;
    vel_int = zeros(3,1);
end

vel_err = vel_sp-vel_curr;


% thrust_sp = vel_err .* para.vel_p + ...
%     para.vel_d .* (-vel_curr) / para.Ts;

thrust_sp = vel_err .* para.vel_p + ...
    para.vel_d .* (vel_prev - vel_curr) / para.Ts + ...
    para.vel_i .* vel_int;

vel_prev = vel_curr;
vel_int = vel_int + vel_err * para.Ts;

% thrust_sp = vel_err.*para.vel_p+para.vel_d.*(-vel_curr)/dt+thrust_int-[0;0;para.thr_hover];
% thrust_int_next = thrust_int+vel_err.*para.vel_i*dt;

% if t ~= 0
%     thrust_sp = velocity_hold(vel_sp, vel_curr, 0, para); 
% else  % 把积分项清零
%     thrust_sp = velocity_hold(vel_sp, vel_curr, 1, para);
% end

% % limit max tilt
% thrust_sp_xy_len = norm(thrust_sp(1:2));
% if thrust_sp_xy_len > 0.01
%     thrust_xy_max = thrust_sp(3) * tan(para.tilt_max);
%     if thrust_sp_xy_len > thrust_xy_max
%         k = thrust_xy_max / thrust_sp_xy_len;
%         thrust_sp(1) = thrust_sp(1) * k;
%         thrust_sp(2) = thrust_sp(2) * k;
%     end
% end

R = q2dcm(q_curr);
R_z = R(:,3);
thrust_body_z = dot(thrust_sp, R_z);
att_sp_thrust = thrust_body_z;

if norm(thrust_sp)==0
    body_z = [0; 0; 1];
else
%     body_z = thrust_sp / norm(thrust_sp);
    body_z = thrust_sp - [0; 0; -para.gravity];
    body_z = body_z / norm(body_z);
end

if body_z(3) < 0
    body_z(3) = -body_z(3);
end

y_C = [-sin(yaw); cos(yaw); 0];
body_x = cross(y_C, body_z);
if body_z(3) < 0
    body_x = -body_x;
end
body_x = body_x / norm(body_x);
body_y = cross(body_z, body_x);
R_sp = [body_x, body_y, body_z];

att_sp = dcm2q(R_sp);
end

function out = sat(in, up_limit, low_limit)
  if in > up_limit
      out = up_limit;
  elseif in < low_limit
      out = low_limit;
  else
      out = in;
  end
end