function [vel_sp] = Position_Controller(pos_sp, pos_curr, para)
% 输入： pos_sp 目标位置；pos_curr 当前位置
% 输出： vel_sp 目标速度

pos_err = pos_sp - pos_curr;
vel_sp = pos_err .* para.pos_p;
end