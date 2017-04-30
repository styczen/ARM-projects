% close all; clear all;
% delete(instrfindall)
% % s = serial('COM3','BaudRate',115200,'Parity','none','DataBits',8,'StopBit',1,'Terminator',64); %assigns the object s to serial port
% s = serial('COM3');
% set(s, 'BaudRate', 115200);
% set(s, 'Parity', 'none');
% set(s, 'DataBits', 8);
% set(s, 'StopBit', 1);
% set(s, 'Terminator', 64); % @ - 64 in ASCII
% fopen(s); %opens the serial port
% %     data = readIMU(s);
% y_lim = [-150, 150];
% i = 0;
% t = 0.01;
% while i<100
%     i=i+1;
%     data = readIMU(s);
%     pitch(i) = data(1);
%     roll(i) = data(2);
%     yaw(i) = data(3);
%     time(i) = i;
%     
%     subplot(3, 1, 1)
%     plot(time, pitch, 'g')
%     axis([time(i)-10, time(i)+10, y_lim])
%     xticks([])
%     drawnow
%     title('Pitch')
% 
%     
%     subplot(3, 1, 2)
%     plot(time, roll, 'b')
%     axis([time(i)-10, time(i)+10, y_lim])
%     xticks([])
%     drawnow
%     title('Roll')
%  
%     
%     subplot(3, 1, 3)
%     plot(time, yaw, 'y')
%     axis([time(i)-10, time(i)+10, y_lim])
%     xticks([])
%     drawnow
%     title('Yaw')
%   
% %     pause(t)
% end
% close all;
% delete(instrfind);
% 
% % fclose(s);
% delete(s);
% clear s;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% clear all;
% close all;
% delete(instrfind);
% 
% %important setting variables
% % buffSize = 100;
% simulation_duration = 10; %time in seconds
% 
% %creating an object imu
% imu = serial('COM3','Baudrate',115200,'Parity','none','DataBits',8,'StopBits',1,'Terminator',64);
% 
% %opening the communication with the object imu
% fopen(imu);
% 
% data = fscanf(imu);
% data = fscanf(imu);
% data = fscanf(imu);
% 
% %%%%% let's star the rotation cube%%%%%%%%%%
% 
% %%% Initialized the cube
% 
% xc=0; yc=0; zc=0;    % coordinated of the center
% Lx=2;                % cube size (length of an edge)
% Ly=1;  
% Lz=0.5;  
% alpha=0.8;           % transparency (max=1=opaque)
% 
% X = [0 0 0 0 0 1; 1 0 1 1 1 1; 1 0 1 1 1 1; 0 0 0 0 0 1];
% Y = [0 0 0 0 1 0; 0 1 0 0 1 1; 0 1 1 1 1 1; 0 0 1 1 1 0];
% Z = [0 0 1 0 0 0; 0 0 1 0 0 0; 1 1 1 0 1 1; 1 1 1 0 1 1];
% 
% C= [0.1 0.5 0.9 0.9 0.1 0.5];   % color/face
% 
% X = Lx*(X-0.5) + xc;
% Y = Ly*(Y-0.5) + yc;
% Z = Lz*(Z-0.5) + zc;
% V=[reshape(X,1,24); reshape(Y,1,24); reshape(Z,1,24)]; %reshape takes the element of X and it fix them in only one colomn (in this case)
% 
% tic; %to count the seconds
% i = 0;
% % while toc<simulation_duration %stop after "simulation duration" seconds
% while i < 100
%     i=i+1;
%     data = readIMU(imu);
%     pitch(i) = data(1);
%     roll = data(2);
%     yaw = 0;
%     dcm_filtered = angle2dcm(pitch(i), roll, yaw, 'YXZ'); %creates the rotation matrix [angoli di eulero -> (z,y,x)]
%     
%     VR_filtered=dcm_filtered*V;
% 
%     XR_filtered=reshape(VR_filtered(1,:),4,6);
%     YR_filtered=reshape(VR_filtered(2,:),4,6);
%     ZR_filtered=reshape(VR_filtered(3,:),4,6);
% 
%     fill3(XR_filtered,YR_filtered,ZR_filtered,C,'FaceAlpha',alpha);
%     xlim([-2 2]);
%     ylim([-2 2]);
%     zlim([-2 2]);
%     box on;
%     grid on
% %     drawnow limitrate
%     pause(0.001)
%    
% end
%     
% close all;
% delete(instrfind);
% 
% % fclose(imu);
% delete(imu);
% clear imu;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
