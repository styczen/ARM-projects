function [out] = readIMU(device) 
% s - serial port device
str = fscanf(device);

for i=1:length(str)
   if str(i)=='#'
      stop1 = i;
      break;
   end
end
pitch = str2double(str(1:stop1-1));

for i=stop1:length(str)
   if str(i)=='$'
      stop2 = i;
      break;
   end
end
roll = str2double(str(stop1+1:stop2-1));

for i=stop2:length(str)
   if str(i)=='@'
       stop3 = i;
       break;
   end
end
yaw = str2double(str(stop2+1:stop3-1));

out = [pitch, roll, yaw];
end