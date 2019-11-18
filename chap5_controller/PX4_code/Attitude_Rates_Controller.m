function [att_control] = ...
    Attitude_Rates_Controller(rates_sp, rates_curr, t, para)
% 输入： rates_sp 目标角速度；rates_curr 当前角速度； 
%        rates_prev 上一时刻的角速度； dt 步长
% 输出： att_control 姿态控制量

persistent rates_prev
persistent rates_int
if t==0
    rates_prev = rates_curr;
    rates_int = zeros(3,1);
end

error = rates_sp - rates_curr;

att_control = para.rate_p .* error + ...   % P
    para.rate_d .* (rates_prev - rates_curr) / para.Ts + ...% D
    para.rate_i .* rates_int + ... % I
    para.rate_ff .* rates_sp;   

rates_prev = rates_curr;
rates_int = rates_int + error * para.Ts;

end