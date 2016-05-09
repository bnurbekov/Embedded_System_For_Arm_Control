%fclose(serialIn);
%delete (serialIn);
clear all;
% Configure the serial port, set the com port and baud rate here
% Use the serial doc to help find the appropriate parameters
serialIn = serial('COM6', 'Baudrate', 115200, 'Terminator', 'LF/CR');

% Open the serial port. Be sure to close it later or you may have trouble
% re-opening
fopen(serialIn);
% Check the serial configuration if you want
disp (serialIn);
% Sample - run in infinie loop

figure
plot(1, 1);
while(1)

    % Wait until a line has at least 24 bytes (in this example, can set to
    % what you need)
    if (serialIn.BytesAvailable >= 24 )
        % Include error checking to avoid it crashing
        try
            % Read the serial buffer, configure the input parsing as 
            % required for your data format
            a = (fscanf(serialIn,'%f, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d'))';
            
            % Show the received and parsed data array
            disp(a);
            
            % Do something with the data, such as call your simulation
            %sim_arm(a(1,1), a(1, 2));
            if (size(a, 2) == 11)
                objects = plot_arm(a(2), a(3), 6, 6);
                drawnow;
                axis([-15 15 -5 15]);
                delete(objects);
            end
        catch err
            rethrow(err);
        end
    end
end


% To disconnect the serial port object from the serial port.
% This will nto happen automaatically in this sample because of the inifinite loop
%fclose(serialIn);

% You can also call this to remove the initialized post
%delete (serialIn);

