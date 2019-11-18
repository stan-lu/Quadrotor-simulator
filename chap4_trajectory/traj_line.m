function desired_state = traj_line(t)

t_max = 8;
t = max(0, min(t, t_max));
t = t/t_max;

pos = 10*t.^3 - 15*t.^4 + 6*t.^5;
vel = (30/t_max)*t.^2 - (60/t_max)*t.^3 + (30/t_max)*t.^4;
acc = (60/t_max^2)*t - (180/t_max^2)*t.^2 + (120/t_max^2)*t.^3;

% output desired state
% desired_state = zeros(11,1);
% desired_state(1:3) = [pos; pos; pos];
% desired_state(4:6) = [vel; vel; vel];
% desired_state(7:9) = [acc; acc; acc];
% desired_state(10) = pos;  % yaw
% desired_state(11) = vel;  % yawdot

desired_state = 1*[pos;pos;pos;vel;vel;vel;acc;acc;acc;pos;vel];

end
