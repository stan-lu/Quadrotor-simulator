function plotmavstatevariables(uu)
%
% modified 12/11/2009 - RB


index_graph = 1;
% process inputs to function
px          = uu(1);             % North position (meters)
py          = uu(2);             % East position (meters)
pz          = uu(3);            % altitude (meters)
u           = uu(4);             % body velocity along x-axis (meters/s)
v           = uu(5);             % body velocity along y-axis (meters/s)
w           = uu(6);             % body velocity along z-axis (meters/s)
phi         = 180/pi*uu(7);      % roll angle (degrees)
theta       = 180/pi*uu(8);      % pitch angle (degrees)
psi         = 180/pi*uu(9);      % yaw angle (degrees)
p           = 180/pi*uu(10);     % body angular rate along x-axis (degrees/s)
q           = 180/pi*uu(11);     % body angular rate along y-axis (degrees/s)
r           = 180/pi*uu(12);     % body angular rate along z-axis (degrees/s)
px_c        = uu(13);            % commanded North position (meters)控制北向位置
py_c        = uu(14);            % commanded East position (meters)
pz_c        = uu(15);
% psi_c       = 180/pi*uu(22);
u_c         = uu(24);
v_c         = uu(25);
w_c         = uu(26);
phi_c       = 180/pi*uu(27);
theta_c     = 180/pi*uu(28);
psi_c       = 180/pi*uu(29);
p_c         = 180/pi*uu(30);
q_c         = 180/pi*uu(31);
r_c         = 180/pi*uu(32);

t           = uu(33);            % simulation time模拟时间
F           = uu(34);
M           = uu(35:37);

phi =  clampWithin180(phi);
theta =  clampWithin180(theta);
psi =  clampWithin180(psi);



% Va          = uu(13);            % airspeed (m/s)
% alpha       = 180/pi*uu(14);     % angle of attack (degrees)攻角
% beta        = 180/pi*uu(15);     % side slip angle (degrees)侧滑角
% wn          = uu(16);            % wind in the North direction
% we          = uu(17);            % wind in the East direction
% wd          = uu(18);            % wind in the Down direction
% pn_c        = uu(19);            % commanded North position (meters)控制北向位置
% pe_c        = uu(20);            % commanded East position (meters)
% h_c         = uu(21);            % commanded altitude (meters)
% Va_c        = uu(22);            % commanded airspeed (meters/s)
% alpha_c     = 180/pi*uu(23);     % commanded angle of attack (degrees)
% beta_c      = 180/pi*uu(24);     % commanded side slip angle (degrees)
% phi_c       = 180/pi*uu(25);     % commanded roll angle (degrees)
% theta_c     = 180/pi*uu(26);     % commanded pitch angle (degrees)
% chi_c       = 180/pi*uu(27);     % commanded course (degrees)
% p_c         = 180/pi*uu(28);     % commanded body angular rate along x-axis (degrees/s)
% q_c         = 180/pi*uu(29);     % commanded body angular rate along y-axis (degrees/s)
% r_c         = 180/pi*uu(30);     % commanded body angular rate along z-axis (degrees/s)
% pn_hat      = uu(31);            % estimated North position (meters)估计北向位置
% pe_hat      = uu(32);            % estimated East position (meters)
% h_hat       = uu(33);            % estimated altitude (meters)
% Va_hat      = uu(34);            % estimated airspeed (meters/s)
% alpha_hat   = 180/pi*uu(35);     % estimated angle of attack (degrees)
% beta_hat    = 180/pi*uu(36);     % estimated side slip angle (degrees)
% phi_hat     = 180/pi*uu(37);     % estimated roll angle (degrees)
% theta_hat   = 180/pi*uu(38);     % estimated pitch angle (degrees)
% chi_hat     = 180/pi*uu(39);     % estimated course (degrees)
% p_hat       = 180/pi*uu(40);     % estimated body angular rate along x-axis (degrees/s)
% q_hat       = 180/pi*uu(41);     % estimated body angular rate along y-axis (degrees/s)
% r_hat       = 180/pi*uu(42);     % estimated body angular rate along z-axis (degrees/s)
%    Vg_hat      = uu(43);            % estimated groundspeed
%    wn_hat      = uu(44);            % estimated North wind
%    we_hat      = uu(45);            % estimated East wind
%    psi_hat     = 180/pi*uu(46);     % estimated heading估计航向
%    bx_hat      = uu(47);            % estimated x-gyro biasX轴偏压估计
%    by_hat      = uu(48);            % estimated y-gyro bias
%    bz_hat      = uu(49);            % estimated z-gyro bias
% delta_e     = 180/pi*uu(50);     % elevator angle (degrees)升降翼变化
% delta_a     = 180/pi*uu(51);     % aileron angle (degrees)副翼变化
% delta_r     = 180/pi*uu(52);     % rudder angle (degrees)方向翼变化
% delta_t     = uu(53);            % throttle setting (unitless)油门


% compute course angle计算航向角
% chi = 180/pi*atan2(Va*sin(psi)+we, Va*cos(psi)+wn);

% define persistent variables 定义持续变量
persistent px_handle
persistent py_handle
persistent pz_handle
persistent u_handle
persistent v_handle
persistent w_handle
persistent phi_handle
persistent theta_handle
persistent psi_handle
persistent p_handle
persistent q_handle
persistent r_handle
persistent F_handle
persistent M_handle


% first time function is called, initialize plot and persistent vars
%调用第一个函数，初始化图和持续变量
if t==0
    
    figure(2*index_graph), clf
    subplot(4,3,1)
    hold on
    px_handle = graph_y_yd(t, px, px_c, 'x/m', []);
    xlabel('t(s)')
    subplot(4,3,2)
    hold on
    py_handle = graph_y_yd(t, py, py_c, 'y/m', []);
    xlabel('t(s)')
    subplot(4,3,3)
    hold on
    pz_handle = graph_y_yd(t, pz, pz_c, 'z/m', []);
    xlabel('t(s)')
    hold on
  
    subplot(4,3,4)
    hold on
    u_handle = graph_y_yd(t, u, u_c, 'u', []);
    xlabel('t(s)')
    subplot(4,3,5)
    hold on
    v_handle = graph_y_yd(t, v, v_c, 'v', []);
    xlabel('t(s)')
    subplot(4,3,6)
    hold on
    w_handle = graph_y_yd(t, w, w_c, 'w', []);
    xlabel('t(s)')
    hold on
    
    subplot(4,3,7)
    hold on
    phi_handle = graph_y_yd(t, phi, phi_c, '\phi/°', []);
    xlabel('t(s)')
    subplot(4,3,8)
    hold on
    theta_handle = graph_y_yd(t, theta, theta_c, '\theta/°', []);
    xlabel('t(s)')
    subplot(4,3,9)
    hold on
    psi_handle = graph_y_yd(t, psi, psi_c, '\psi/°', []);
    xlabel('t(s)')
    hold on
    
    subplot(4,3,10)
    hold on
    p_handle = graph_y_yd(t, p, p_c, 'p', []);
    xlabel('t(s)')
    subplot(4,3,11)
    hold on
    q_handle = graph_y_yd(t, q, q_c, 'q', []);
    xlabel('t(s)')
    subplot(4,3,12)
    hold on
    r_handle = graph_y_yd(t, r, r_c, 'r', []);
    xlabel('t(s)')
    hold on
    
    figure(2*index_graph+1), clf
    subplot(2,3,1:3)
    hold on
    F_handle = graph_y(t, F, [], 'F');
    xlabel('t(s)');
    subplot(2,3,4)
    hold on
    M_handle(1) = graph_y(t, M(1), [], 'M_1');
    xlabel('t(s)');
    subplot(2,3,5)
    hold on
    M_handle(2) = graph_y(t, M(2), [], 'M_2');
    xlabel('t(s)');
    subplot(2,3,6)
    hold on
    M_handle(3) = graph_y(t, M(3), [], 'M_3');
    xlabel('t(s)');
    
    % at every other time step, redraw state variables
    %在每隔一个时间步，重绘状态变量
else
    graph_y_yd(t, px, px_c, 'x', px_handle);
    graph_y_yd(t, py, py_c, 'y', py_handle);
    graph_y_yd(t, pz, pz_c, 'z', pz_handle);
    graph_y_yd(t,  u, u_c, 'u', u_handle);
    graph_y_yd(t,  v, v_c, 'v', v_handle);
    graph_y_yd(t,  w, w_c, 'w', w_handle);
    graph_y_yd(t, phi, phi_c, '\phi', phi_handle);
    graph_y_yd(t, theta, theta_c, '\theta', theta_handle);
    graph_y_yd(t, psi, psi_c, '\psi', psi_handle);
    graph_y_yd(t, p, p_c, 'p', p_handle);
    graph_y_yd(t, q, q_c, 'q', q_handle);
    graph_y_yd(t, r, r_c, 'r', r_handle);
    graph_y(t, F, F_handle, 'F');
    graph_y(t, M(1),  M_handle(1), 'M_1');
    graph_y(t, M(2),  M_handle(2), 'M_2');
    graph_y(t, M(3),  M_handle(3), 'M_3');
end

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% graph y with lable mylabel
function handle = graph_y(t, y, handle, lab)

if isempty(handle)
    handle    = plot(t,y,'b');
    ylabel(lab)
    set(get(gca, 'YLabel'),'Rotation',0.0);
else
    set(handle,'Xdata',[get(handle,'Xdata'),t]);
    set(handle,'Ydata',[get(handle,'Ydata'),y]);
    %drawnow
end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% graph y and yd with lable mylabel
function handle = graph_y_yd(t, y, yd, lab, handle)

if isempty(handle)
    handle(1)    = plot(t,y,'b');
    handle(2)    = plot(t,yd,'r:');
    ylabel(lab)
    set(get(gca, 'YLabel'),'Rotation',0.0);
%     legend('实际值','期望值');
else
    set(handle(1),'Xdata',[get(handle(1),'Xdata'),t]);
    set(handle(1),'Ydata',[get(handle(1),'Ydata'),y]);
    set(handle(2),'Xdata',[get(handle(2),'Xdata'),t]);
    set(handle(2),'Ydata',[get(handle(2),'Ydata'),yd]);
    %drawnow
end
end



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% plot the variable y in blue, its estimated value yhat in green, and its
% desired value yd in red, lab is the label on the graph
%绘制变量y为蓝色，其估计值为绿色，其期望值YD为红色，Lab是图上的标记。
function handle = graph_y_yhat_yd(t, y, yhat, yd, lab, handle)

if isempty(handle),
    handle(1)   = plot(t,y,'b');
    handle(2)   = plot(t,yhat,'g--');
    handle(3)   = plot(t,yd,'r-.');
    ylabel(lab,'FontSize',30)
    set(get(gca,'YLabel'),'Rotation',0.0);
else
    set(handle(1),'Xdata',[get(handle(1),'Xdata'),t]);
    set(handle(1),'Ydata',[get(handle(1),'Ydata'),y]);
    set(handle(2),'Xdata',[get(handle(2),'Xdata'),t]);
    set(handle(2),'Ydata',[get(handle(2),'Ydata'),yhat]);
    set(handle(3),'Xdata',[get(handle(3),'Xdata'),t]);
    set(handle(3),'Ydata',[get(handle(3),'Ydata'),yd]);
    %drawnow
end
end

%
%=============================================================================
% sat
% saturates the input between high and low饱和高与低之间的输入
%=============================================================================
%
function out=sat(in, low, high)

if in < low,
    out = low;
elseif in > high,
    out = high;
else
    out = in;
end
end
% end sat

function out = clampWithin180(in)

out = in;

while out > 180
    out = out - 360;
end
while out < -180
    out = out + 360;
end

end

