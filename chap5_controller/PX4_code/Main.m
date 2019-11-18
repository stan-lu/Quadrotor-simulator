clear
clc

timeSpan=40;  % 仿真时长
dt=0.1;       % 步长

time=0:dt:timeSpan;

% att_sp=euler2q(0,0,0);  % 目标姿态四元数 
pos_sp=[1;0;0];            % 目标位置(NED)
yaw_des=0;                 % 目标偏航角
euler_ini=[0;0;0];         % 初始姿态 roll pitch yaw
rates_ini=[0;0;0];         % 初始角速度
pos_ini=[0;0;0];           % 初始位置
vel_ini=[0;0;0];           % 初始速度
thrust=0.5;                % 油门量 0~1

euler_log=zeros(3,size(time,2)+1); % 记录每一时刻的姿态角
euler_log(:,1)=euler_ini;
rates_log=zeros(3,size(time,2)+1); % 记录每一时刻的角速度（体坐标系下）
rates_log(:,1)=rates_ini;
pos_log=zeros(3,size(time,2)+1);   % 记录每一时刻的位置
pos_log(:,1)=pos_ini;
vel_log=zeros(3,size(time,2)+1);   % 记录每一时刻的速度
vel_log(:,1)=vel_ini;
att_sp_log=zeros(4,size(time,2)+1); % 记录每一时刻的目标姿态
vel_sp_log=zeros(3,size(time,2)+1); % 记录每一时刻的目标速度
rates_sp_log=zeros(3,size(time,2)+1); % 记录每一时刻的目标角速度
att_control_log=zeros(4,size(time,2)+1); % 记录每一时刻的姿态控制量
thrust_int_log=zeros(3,size(time,2)+1); % 记录 thrust 的积分值

for i=1:size(time,2)
    % 位置内环
    vel_sp=Position_Controller(pos_sp,pos_log(:,i));
    vel_sp_log(:,i+1)=vel_sp;
    
    % 位置外环
    q_cur=euler2q(euler_log(1,i),euler_log(2,i),euler_log(3,i));
    [~,att_sp,thrust_int_,att_sp_thrust]=Velocity_Controller(vel_sp,vel_log(:,i),dt,thrust_int_log(:,i),q_cur,yaw_des);
    thrust_int_log(:,i+1)=thrust_int_;
    att_sp_log(:,i+1)=att_sp;
    
%     att_sp=euler2q(0,pi/12,0);
    % 姿态内环
    rates_sp=Attitude_Controller(att_sp,q_cur,0);
    rates_sp_log(:,i+1)=rates_sp;
    % 姿态外环
    if i==1
        rates_prev=rates_ini;
    else
       rates_prev=rates_log(:,i-1); 
    end
    att_control=Attitude_Rates_Controller(rates_sp,rates_log(:,i),rates_prev,dt);
%     att_control(4)=att_sp_thrust;
    att_control(4)=thrust;
    att_control_log(:,i+1)=att_control;
    
    % 得到每个电机的转速
    motor_speed=Mixer(att_control);
    % 得到作用在四旋翼上的力和力矩
    [F,M]=MotorModel(motor_speed);
    % 更新位置和姿态
    y0=[pos_log(:,i);vel_log(:,i);euler_log(:,i);rates_log(:,i)];
    [t,y]=ode45(@(t,y) DynamicModel(t,y,F,M),[0 dt],y0);
    
    pos_log(1:2,i+1)=y(end,1:2)';
    pos_log(3,i+1)=0;
    vel_log(:,i+1)=y(end,4:6)';
    euler_log(:,i+1)=y(end,7:9)';
    rates_log(:,i+1)=y(end,10:12)';
   
end

% 画图
axis_x=(1:size(time,2)+1)*dt;
figure(1)
% 姿态角
subplot(2,2,1)
plot(axis_x,euler_log(1,:),axis_x,euler_log(2,:),axis_x,euler_log(3,:));
legend('roll/rad','pitch/rad','yaw/rad');
title('姿态');
% 位置（NED）
subplot(2,2,2)
plot(axis_x,pos_log(1,:),axis_x,pos_log(2,:),axis_x,pos_log(3,:),axis_x,zeros(1,size(axis_x,2))+1);
legend('x','y','z','目标');
title('位置');
% 目标姿态
euler=zeros(3,i+1);
for j=1:i+1
   euler(:,j)=q2euler(att_sp_log(:,j));
end
subplot(2,2,3)
plot(axis_x,euler(1,:),axis_x,euler(2,:),axis_x,euler(3,:));
title('目标姿态');
legend('roll','pitch','yaw');
subplot(2,2,4)
plot(axis_x,vel_sp_log(1,:),axis_x,vel_sp_log(2,:),axis_x,vel_sp_log(3,:));
legend('x','y','z');
title('目标速度');

figure(2)
subplot(2,2,1)
plot(axis_x,vel_log(1,:),axis_x,vel_log(2,:),axis_x,vel_log(3,:));
legend('x','y','z');
title('速度');
subplot(2,2,3)
plot(axis_x,vel_sp_log(1,:),axis_x,vel_sp_log(2,:),axis_x,vel_sp_log(3,:));
legend('x','y','z');
title('目标速度');
subplot(2,2,2)
plot(axis_x,rates_log(1,:),axis_x,rates_log(2,:),axis_x,rates_log(3,:));
legend('x','y','z');
title('角速度');
subplot(2,2,4)
plot(axis_x,rates_sp_log(1,:),axis_x,rates_sp_log(2,:),axis_x,rates_sp_log(3,:));
legend('x','y','z');
title('目标角速度');

% figure(3)
% plot(axis_x,att_control_log(1,:),axis_x,att_control_log(2,:),axis_x,att_control_log(3,:),axis_x,att_control_log(4,:));
% legend('roll','pitch','yaw','thrust');
% title('姿态控制量');
