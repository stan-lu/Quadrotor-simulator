function [rates_sp]=Attitude_Controller_v1_8_0(att_sp,cur_state,yaw_sp_move_rate, para)
% 输入： att_sp 目标姿态四元数;  cur_state  当前姿态四元数
% 输出： rats_sp 目标角速度（3 X 1）
if 1
    % 1 表示使用的 v1.8.0 版本的算法
    % 0 表示使用的 v1.5.5 版本的算法
    attitude_gain = para.att_p;
    roll_pitch_gain = (attitude_gain(1) + attitude_gain(2)) / 2;
    yaw_w = constrain(attitude_gain(3) / roll_pitch_gain, 0, 1);
    attitude_gain(3) = roll_pitch_gain;
    
    qd = att_sp / norm(att_sp);
    q = cur_state / norm(cur_state);
    
    e_z = q2dcm_z(q);
    e_z_d = q2dcm_z(qd);
    qd_red = two_Vec2q(e_z, e_z_d);
    
    if abs(qd_red(2)) > (1 - 1e-5) || abs(qd_red(3)) > (1 - 1e-5)
        qd_red = qd;
    else
        qd_red = quat_multi(qd_red, q);
    end
    
    q_mix = quat_multi(quat_inver(qd_red), qd);
    q_mix = q_mix * signNoZero(q_mix(1));
    
    q_mix(1) = constrain(q_mix(1), -1, 1);
    q_mix(4) = constrain(q_mix(4), -1, 1);
    qd = quat_multi(qd_red, ...
        [cos(yaw_w * acos(q_mix(1))); 0; 0; sin(yaw_w * asin(q_mix(4)))]);
    
    qe = quat_multi(quat_inver(q), qd);
    
    eq = 2 * signNoZero(qe(1)) * qe(2:end);
    
    rates_sp = eq .* attitude_gain;
    
    rates_sp = rates_sp + q2dcm_z(quat_inver(q)) * yaw_sp_move_rate;
    
else
    
    
    R_sp = q2dcm(att_sp);  % 把目标姿态转成矩阵形式
    R = q2dcm(cur_state);
    
    R_z = R(:, 3);
    R_sp_z = R_sp(:, 3);
    
    R_tran = R';
    e_R = R_tran * (cross(R_z, R_sp_z));
    e_R_z_sin = norm(e_R);  % |e_R| = sin(R_z,R_sp_z)
    e_R_z_cos = R_z' * R_sp_z; % R_z 和 R_sp_z 都是单位向量
    
    yaw_w = R_sp(3,3) * R_sp(3,3);  % yaw 控制的权重  z轴夹角越大，权重就越小，yaw控制的重要性越低
    % 意味着先注重 pitch-roll 的控制；反之，则认为 z轴
    % 已经对齐了，要开始对 yaw 进行控制了
    
    % R_rp=zeros(3,3);
    
    if e_R_z_sin > 0
        e_R_z_angle = atan2(e_R_z_sin, e_R_z_cos);
        e_R_z_axis = e_R / e_R_z_sin;
        
        e_R = e_R_z_axis * e_R_z_angle;
        
        e_R_cp = zeros(3,3);
        e_R_cp(1,2) = -e_R_z_axis(3);
        e_R_cp(1,3) = e_R_z_axis(2);
        e_R_cp(2,1) = e_R_z_axis(3);
        e_R_cp(2,3) = -e_R_z_axis(1);
        e_R_cp(3,1) = -e_R_z_axis(2);
        e_R_cp(3,2) = e_R_z_axis(1);
        
        R_rp = R * ...
            (eye(3) + e_R_cp * e_R_z_sin + e_R_cp * e_R_cp * (1 - e_R_z_cos));
        
    else
        R_rp = R;
    end
    
    R_sp_x = R_sp(:,1);
    R_rp_x = R_rp(:,1);
    e_R(3) = atan2(cross(R_rp_x, R_sp_x)' * R_sp_z, R_rp_x' * R_sp_x) * yaw_w;
    
    if e_R_z_cos < 0
        q_error = dcm2q(R_tran * R_sp);
        if q_error(1) >= 0
            e_R_d = q_error(2:4, 1) * 2;
        else
            e_R_d = q_error(2:4, 1) * -2;
        end
        
        direct_w = e_R_z_cos * e_R_z_cos * yaw_w;
        e_R = e_R * (1 - direct_w) + e_R_d * direct_w;
    end
    
    rates_sp = para.att_p .* e_R;
    
    for i=1:3
        rates_sp(i) = ...
            constrain(rates_sp(i), -para.ratesMax(i), para.ratesMax(i));
    end
    
    rates_sp(3) = rates_sp(3) + yaw_sp_move_rate * yaw_w * para.yaw_ff;
    
end

end

function r = quat_multi(p, q)
% return r = p * q

r = zeros(4,1);
r(1) = p(1) * q(1) - p(2) * q(2) - p(3) * q(3) - p(4) * q(4);
r(2) = p(1) * q(2) + p(2) * q(1) + p(3) * q(4) - p(4) * q(3);
r(3) = p(1) * q(3) - p(2) * q(4) + p(3) * q(1) + p(4) * q(2);
r(4) = p(1) * q(4) + p(2) * q(3) - p(3) * q(2) + p(4) * q(1);

end

function r = quat_inver(q)

normSq = dot(q, q);
r = [q(1); -q(2); -q(3); -q(4)] / normSq;

end

function bool = signNoZero(val)

bool = (val >= 0) - (val < 0);

end