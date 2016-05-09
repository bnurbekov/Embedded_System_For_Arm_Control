clear all;
% Configure the serial port, set the com port and baud rate here
% Use the serial doc to help find the appropriate parameters
serialIn = serial('COM6', 'Baudrate', 115200, 'Terminator', 'LF/CR');

% Open the serial port. Be sure to close it later or you may have trouble
% re-opening
fopen(serialIn);
% Check the serial configuration if you want
disp (serialIn);

fprintf(serialIn,'%s\r','12 24')