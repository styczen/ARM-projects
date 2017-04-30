% anim_block_translate.m

clear; close all;

% Serial settings
delete(instrfindall)
s = serial('COM3');
set(s, 'BaudRate', 115200);
set(s, 'Parity', 'none');
set(s, 'DataBits', 8);
set(s, 'StopBit', 1);
set(s, 'Terminator', 64); % @ - 64 in ASCII
fopen(s); %opens the serial port

% Block specification
Lx = 10;
Ly = 6;
Lz = 1.5;

% Motion data
t = [0:0.005:1]';                % Time data
r = [0*t, 0*t, 0*t]; % Position data
% A = [0*t, 0*t, 0*t];             % Orientation data (x-y-z Euler angle)

n_time = length(t);

h = patch(PatchData_X(:,:,i_time),PatchData_Y(:,:,i_time),PatchData_Z(:,:,i_time),'y');
% Compute propagation of vertices and patches
for i_time=1:n_time
    data = readIMU(s);
    pitch = data(1);
    roll = data(2);
    yaw = data(3);
    A(i_time, :) = [pitch, roll, yaw];
    
    R = Euler2R(A(i_time,:));
    VertexData(:,:,i_time) = GeoVerMakeBlock(r(i_time,:),R,[Lx,Ly,Lz]);
    [X,Y,Z] = GeoPatMakeBlock(VertexData(:,:,i_time));
    PatchData_X(:,:,i_time) = X;
    PatchData_Y(:,:,i_time) = Y;
    PatchData_Z(:,:,i_time) = Z;
    axis vis3d equal;
    grid on;
    xlim([-15,15]);
    ylim([-15,15]);
    zlim([-15,15]);
    
    set(h,'XData',PatchData_X(:,:,i_time));
    set(h,'YData',PatchData_Y(:,:,i_time));
    set(h,'ZData',PatchData_Z(:,:,i_time));
    drawnow;
end

% Draw initial figure
% figure(1);
% h = patch(PatchData_X(:,:,1),PatchData_Y(:,:,1),PatchData_Z(:,:,1),'y');
% set(h,'FaceLighting','phong','EdgeLighting','phong');
% set(h,'EraseMode','normal');

% Axes settings
% xlabel('x','FontSize',14);
% ylabel('y','FontSize',14);
% zlabel('z','FontSize',14);
% set(gca,'FontSize',14);
% axis vis3d equal;
% view([-37.5,30]);
% camlight;
% grid on;
% xlim([-15,15]);
% ylim([-15,15]);
% zlim([-15,15]);

% Animation Loop
% for i_time=1:n_time
%     
%     set(h,'XData',PatchData_X(:,:,i_time));
%     set(h,'YData',PatchData_Y(:,:,i_time));
%     set(h,'ZData',PatchData_Z(:,:,i_time));
%     drawnow;
% end

% close all;
delete(instrfind);
% fclose(s);
delete(s);
clear s;