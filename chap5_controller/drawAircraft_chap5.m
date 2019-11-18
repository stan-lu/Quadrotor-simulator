
function drawAircraft(uu,P)

% process inputs to function

px_c     = uu(1);
py_c     = uu(2);
pz_c     = uu(3);
t        = uu(12);       % time
px       = uu(13);       % inertial x position
py       = uu(14);       % inertial y position
pz       = uu(15);
phi      = uu(19);       % roll angle
theta    = uu(20);       % pitch angle
psi      = uu(21);       % yaw angle


% define persistent variables
persistent vehicle_handle;
persistent Vertices
persistent Faces
persistent facecolors
persistent field_view_handle

persistent h_pos_handle;  % position history handle
persistent h_pos_de_handle; % position demand history handle


% first time function is called, initialize plot and persistent vars
if t==0
    figure(1), clf
    h_pos_handle = drawTrajectory(px, py, pz, 'b', []);
    hold on
    h_pos_de_handle = drawTrajectory(px_c, py_c, pz_c, 'r:', []');
    hold on
    
%     legend('实际路径', '期望路径');
    
    [Vertices,Faces,facecolors] = defineQuadrotorBody;
    vehicle_handle(1) = drawVehicleBody(Vertices,Faces,facecolors,...
        px,py,pz,phi,theta,psi,...
        [],'normal');
    
    title('Vehicle')
    xlabel('x')
    ylabel('y')
    zlabel('z')
    set(get(gca, 'ZLabel'),'Rotation',0.0);
    view(32,47)  % set the vieew angle for figure
%     axis([-0.3,1.5,-0.3,1.5,0,1.5]);
        axis([-110,20,-66,60,-1,30]);
    grid on
    %     hold on
    
        Vert_field_view = findFieldOfView(px,py,pz,phi,theta,psi,P);
        [field_view_handle(1),field_view_handle(2)] = ...
            drawFieldOfView(px,py,pz,Vert_field_view,[],[]);
    
    
    
    % at every other time step, redraw base and rod
else
    drawVehicleBody(Vertices,Faces,facecolors,...
        px,py,pz,phi,theta,psi,...
        vehicle_handle);
    
    drawTrajectory(px, py, pz, 'b', h_pos_handle);
    
    drawTrajectory(px_c, py_c, pz_c, 'r:', h_pos_de_handle);
    
        Vert_field_view = findFieldOfView(px,py,pz,phi,theta,psi,P);
        drawFieldOfView(px,py,pz,Vert_field_view,field_view_handle(1),field_view_handle(2));
end
end


%=======================================================================
% drawVehicle
% return handle if 3rd argument is empty, otherwise use 3rd arg as handle
%=======================================================================
%
function handle = drawVehicleBody(V,F,patchcolors,...
    pn,pe,pd,phi,theta,psi,...
    handle,mode)
V = rotate(V, phi, theta, psi);  % rotate vehicle
V = translate(V, pn, pe, pd);  % translate vehicle
% transform vertices from NED to XYZ (for matlab rendering)
%   R = [...
%       0, 1, 0;...
%       1, 0, 0;...
%       0, 0, -1;...
%       ];
%   V = R*V;

if isempty(handle)
    handle = patch('Vertices', V', 'Faces', F,...
        'FaceVertexCData',patchcolors,...
        'FaceColor','flat',...
        'EraseMode', mode);
else
    set(handle,'Vertices',V','Faces',F);
    drawnow
end
end

%%%%%%%%%%%%%%%%%%%%%%%
function pts=rotate(pts,phi,theta,psi)

% define rotation matrix (right handed)
R_roll = [...
    1, 0, 0;...
    0, cos(phi), sin(phi);...
    0, -sin(phi), cos(phi)];
R_pitch = [...
    cos(theta), 0, -sin(theta);...
    0, 1, 0;...
    sin(theta), 0, cos(theta)];
R_yaw = [...
    cos(psi), sin(psi), 0;...
    -sin(psi), cos(psi), 0;...
    0, 0, 1];
R = R_pitch*R_roll*R_yaw;
% note that R above either leaves the vector alone or rotates
% a vector in a left handed rotation.  We want to rotate all
% points in a right handed rotation, so we must transpose
R = R';

% rotate vertices
pts = R*pts;

end
% end rotateVert

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% translate vertices by pn, pe, pd
function pts = translate(pts,pn,pe,pd)

pts = pts + repmat([pn;pe;pd],1,size(pts,2));

end

% end translate


%=======================================================================
% defineVehicleBody
%=======================================================================
function [V,F,facecolors] = defineVehicleBody

% Define the vertices (physical location of vertices
V = 10*[...
    1, 0, 0;...   % pt 1
    -1, -2, 0;... % pt 2
    0, 0, 0;...   % pt 3
    -1, 2, 0;...  % pt 4
    0, 0, -1;...  % pt 5
    ]';

% define faces as a list of vertices numbered above
F = [...
    1, 2, 3;...  % left wing
    1, 3, 4;...  % right wing
    1, 3, 5;...  % tail
    ];

% define colors for each face
myred = [1, 0, 0];
mygreen = [0, 1, 0];
myblue = [0, 0, 1];
myyellow = [1, 1, 0];
mycyan = [0, 1, 1];

facecolors = [...
    mygreen;...    % left wing
    mygreen;...    % right wing
    myblue;...     % tail
    ];
end
% define aircraft vertices and faces
function [V,F,colors] = defineAircraftBody

% parameters for drawing aircraft
% scale size
size = 2;
fuse_l1    = 7;
fuse_l2    = 4;
fuse_l3    = 15;
fuse_w     = 2;
wing_l     = 6;
wing_w     = 20;
tail_l     = 3;
tail_h     = 3;
tailwing_w = 10;
tailwing_l = 3;
% colors
red     = [1, 0, 0];
green   = [0, 1, 0];
blue    = [0, 0, 1];
yellow  = [1,1,0];
magenta = [0, 1, 1];


% define vertices and faces for aircraft
V = [...
    fuse_l1,             0,             0;...        % point 1
    fuse_l2,            -fuse_w/2,     -fuse_w/2;... % point 2
    fuse_l2,             fuse_w/2,     -fuse_w/2;... % point 3
    fuse_l2,             fuse_w/2,      fuse_w/2;... % point 4
    fuse_l2,            -fuse_w/2,      fuse_w/2;... % point 5
    -fuse_l3,             0,             0;...        % point 6
    0,                   wing_w/2,      0;...        % point 7
    -wing_l,              wing_w/2,      0;...        % point 8
    -wing_l,             -wing_w/2,      0;...        % point 9
    0,                  -wing_w/2,      0;...        % point 10
    -fuse_l3+tailwing_l,  tailwing_w/2,  0;...        % point 11
    -fuse_l3,             tailwing_w/2,  0;...        % point 12
    -fuse_l3,            -tailwing_w/2,  0;...        % point 13
    -fuse_l3+tailwing_l, -tailwing_w/2,  0;...        % point 14
    -fuse_l3+tailwing_l,  0,             0;...        % point 15
    -fuse_l3+tailwing_l,  0,             -tail_h;...  % point 16
    -fuse_l3,             0,             -tail_h;...  % point 17
    ]';

F = [...
    1,  2,  3,  1;... % nose-top
    1,  3,  4,  1;... % nose-left
    1,  4,  5,  1;... % nose-bottom
    1,  5,  2,  1;... % nose-right
    2,  3,  6,  2;... % fuselage-top
    3,  6,  4,  3;... % fuselage-left
    4,  6,  5,  4;... % fuselage-bottom
    2,  5,  6,  2;... % fuselage-right
    7,  8,  9, 10;... % wing
    11, 12, 13, 14;... % tailwing
    6, 15, 17, 17;... % tail
    
    ];

colors = [...
    yellow;... % nose-top
    yellow;... % nose-left
    yellow;... % nose-bottom
    yellow;... % nose-right
    blue;... % fuselage-top
    blue;... % fuselage-left
    red;... % fuselage-bottom
    blue;... % fuselage-right
    green;... % wing
    green;... % tailwing
    blue;... % tail
    ];

V = size*V;   % rescale vertices

end

function [V,F,colors] = defineQuadrotorBody
% scale size
size = 1;
arm_length = 0.1; % m
arm_width =0.01; % m
% colors
red  = [1,0,0];
blue = [0,0,1];
green = [0,1,0];

% points
V = [...
    arm_length,     -arm_width,     arm_width; % point 1
    arm_length,      arm_width,     arm_width; % point 2
    arm_length,      arm_width,    -arm_width; % point 3
    arm_length,     -arm_width,    -arm_width; % point 4
    -arm_length,     -arm_width,     arm_width; % point 5
    -arm_length,      arm_width,     arm_width; % point 6
    -arm_length,     -arm_width,    -arm_width; % point 7
    -arm_length,      arm_width,    -arm_width; % point 8
    -arm_width,    -arm_length,     arm_width; % point 9
    arm_width,    -arm_length,     arm_width; % point 10
    arm_width,    -arm_length,    -arm_width; % point 11
    -arm_width,    -arm_length,    -arm_width; % point 12
    -arm_width,     arm_length,     arm_width; % point 13
    arm_width,     arm_length,     arm_width; % point 14
    -arm_width,     arm_length,    -arm_width; % point 15
    arm_width,     arm_length,    -arm_width; % point 16
    ]';

F = [
    1,  2,  3,  4;
    1,  5,  6,  2;
    1,  4,  7,  5;
    4,  3,  8,  7; % bottom
    2,  6,  8,  3;
    5,  7,  8,  6;
    9, 13, 14, 10;
    10, 14, 16, 11;
    11, 16, 15, 12; % bottom
    9, 12, 15, 13;
    9, 10, 11, 12;
    13, 15, 16, 14;
    ];

colors = [
    blue;
    green;
    blue;
    red;   % bottom
    blue;
    blue;
    blue;
    blue;
    red;  % bottom
    blue;
    blue;
    blue
    ];

V = size * V;
end

%=======================================================================
% drawTrajectory
%=======================================================================
function handle = drawTrajectory(x, y, z, color, handle)

if isempty(handle)
    handle = plot3(x, y, z, color);
else
    set(handle,'Xdata',[get(handle,'Xdata'),x]);
    set(handle,'Ydata',[get(handle,'Ydata'),y]);
    set(handle,'Zdata',[get(handle,'Zdata'),z]);
end

end
%=======================================================================
% findFieldOfView
%=======================================================================
function V = findFieldOfView(x,y,z,phi,theta,psi,P)

% V = zeros(3,4); % 4个点，每个点3个坐标

tx = tan(P.theta_x / 2);
ty = tan(P.theta_y / 2);
cp = cos(phi);
sp = sin(phi);
ct = cos(theta);
st = sin(theta);
cs = cos(psi);
ss = sin(psi);

rotation = [cs*ct - sp*ss*st, -cp*ss, cs*st + ct*sp*ss;
    ct*ss + cs*sp*st, cp*cs, ss*st - cs*ct*sp;
    -cp*st, sp, cp*ct];

V = [tx,  ty, -1;
    tx, -ty, -1;
    -tx,  ty, -1;
    -tx, -ty, -1]';

V = rotation * V * z;

V = V + repmat([x;y;z],1,4);
end

%=======================================================================
% drawFieldOfView
%=======================================================================
function [handle,handle_2] = drawFieldOfView(x,y,z,Vert,handle,handle_2)

XYZ = [
    Vert(:,1)';
    Vert(:,2)';
    Vert(:,4)';
    Vert(:,3)';
    Vert(:,1)'
    ]';

XYZ_2 = [
    Vert(:,1)';
    x,y,z;
    Vert(:,2)';
    x,y,z;
    Vert(:,3)';
    x,y,z;
    Vert(:,4)'
    ]';

if isempty(handle)
    handle   = plot3(XYZ(1,:),XYZ(2,:),XYZ(3,:),'k');
    handle_2 = plot3(XYZ_2(1,:),XYZ_2(2,:),XYZ_2(3,:),':','Color',[0.8 0.8 0.8]);
else
    set(handle,'XData',XYZ(1,:),'YData',XYZ(2,:),'ZData',XYZ(3,:));
    set(handle_2,'XData',XYZ_2(1,:),'YData',XYZ_2(2,:),'ZData',XYZ_2(3,:));
    drawnow
end

end