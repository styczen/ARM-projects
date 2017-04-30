% make_block.m

clear; close all;

% Block specification
r = [1,1,1];          % Reference position
A = [-pi/3, 0, pi/6]; % Reference orientation (x-y-z Euler angle)

Lx = 0.15;
Ly = 0.05;
Lz = 0.30;

% Euler angle -> Orientation matrix
R = Euler2R(A);

% Vertices
VertexData = GeoVerMakeBlock(r,R,[Lx,Ly,Lz]);

% Patches
[PatchData_X,PatchData_Y,PatchData_Z] = GeoPatMakeBlock(VertexData);

% Draw patches
figure(1);
h = patch(PatchData_X,PatchData_Y,PatchData_Z,'y');
set(h,'FaceLighting','phong','EdgeLighting','phong');
set(h,'EraseMode','normal');

% Axes settings
xlabel('x','FontSize',14);
ylabel('y','FontSize',14);
zlabel('z','FontSize',14);
set(gca,'FontSize',14);
axis vis3d equal;
view([-37.5,30]);
camlight;
grid on;
xlim([0.8,1.3]);
ylim([0.9,1.4]);
zlim([0.8,1.3]);