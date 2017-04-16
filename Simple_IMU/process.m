close all; clear all;
delete(instrfindall)
s = serial('COM3'); %assigns the object s to serial port
set(s, 'BaudRate', 9600);
set(s, 'Parity', 'none');
set(s, 'DataBits', 8);
set(s, 'StopBit', 1);
set(s, 'Terminator', 64); % @ - 64 in ASCII
fopen(s); %opens the serial port

data = zeros(10, 2);
t=1:10;

for i=1:10
    data(i,:) = readIMU(s);
    pause(0.0001)
end
figure(1)
plot(t, data(:,1))




fclose(s);
delete(s);
clear s;