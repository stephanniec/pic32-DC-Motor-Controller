function client(port)
%   provides a menu for accessing PIC32 motor control functions
%
%   client(port)
%
%   Input Arguments:
%       port - the name of the com port.  This should be the same as what
%               you use in screen or putty in quotes ' '
%
%   Example:
%       client('/dev/ttyUSB0') (Linux/Mac)
%       client('COM3') (PC)
%
%   For convenience, you may want to change this so that the port is hardcoded.
   
% Opening COM connection
if ~isempty(instrfind)
    fclose(instrfind);
    delete(instrfind);
end

fprintf('Opening port %s....\n',port);

% settings for opening the serial port. baud rate 230400, hardware flow control
% wait up to 120 seconds for data before timing out
mySerial = serial(port, 'BaudRate', 230400, 'FlowControl', 'hardware','Timeout',10); 
% opens serial connection
fopen(mySerial);
% closes serial port when function exits
clean = onCleanup(@()fclose(mySerial));                                 

has_quit = false;
% menu loop
while ~has_quit
    fprintf('PIC32 MOTOR DRIVER INTERFACE\n\n');
    % display the menu options; this list will grow
    fprintf('     a: Read current sensor (ADC counts)\n'    );
    fprintf('     b: Read current sensor (mA)\n'            );
    fprintf('     c: Read encoder (counts)\n'               );
    fprintf('     d: Read encoder (deg)\n'                  );
    fprintf('     e: Reset encoder\n'                       );
    fprintf('     f: Set PWM (-100 to 100)\n'               );
    fprintf('     g: Set current gains\n'                   );
    fprintf('     h: Get current gains\n'                   );
    fprintf('     i: Set position gains\n'                  );
    fprintf('     j: Get position agins\n'                  );
    fprintf('     k: Test current control\n'                );
    fprintf('     l: Go to angle (deg)\n'                   );
    fprintf('     p: Unpower the motor\n'                   );
    fprintf('     q: Quit client\n'                         );
    fprintf('     r: Get mode\n'                            );
   
        % read the user's choice
    selection = input('\nENTER COMMAND: ', 's');
     
    % send the command to the PIC32
    fprintf(mySerial,'%c\n',selection);
    
    % take the appropriate action
    switch selection     
        case 'a'                         % Read current sensor (ADC counts)
            adc_clkticks = fscanf(mySerial, '%d');
            fprintf('The motor current is %d ADC counts.\n', adc_clkticks);
        case 'b'                         % Read current sensor (mA)
            adc_mamps = fscanf(mySerial, '%f');
            fprintf('The motor current is %f mA.\n', adc_mamps); 
        case 'c'                         % Read encoder counts
            counts = fscanf(mySerial, '%d');
            fprintf('The motor angle is %d counts.\n', counts);
        case 'd'                         % Read encoder deg
            deg_100x = fscanf(mySerial, '%d');
            fprintf('The motor angle is %0.1f degrees.\n', deg_100x/100.0);
        case 'e'                         % Reset encoder
            fprintf('Encoder reset.\n');
            fprintf('Currently at new zero position.\n');
        case 'f'                         % Set PWM (-100 to 100)
            dutycycle = input('Enter desired PWM value (-100 to 100): ');
            fprintf(mySerial, '%d\n', dutycycle);
            if (dutycycle < 0)
                fprintf('PWM set to %d percent in the counterclockwise direction.\n', dutycycle);
            else
                fprintf('PWM set to %d percent in the clockwise direction.\n', dutycycle);
            end
        case 'g'                        % Set current gains as floats
            kp_set = input('Enter your desired Kp current gain: ');
            fprintf(mySerial, '%f\n', kp_set);
            ki_set = input('Enter you desired Ki current gain: ');
            fprintf(mySerial, '%f\n', ki_set);
            fprintf('Sending Kp = %.3f and Ki = %.3f to the current controller.\n', kp_set, ki_set);
        case 'h'                        % Get current gains
            Kp = fscanf(mySerial, '%f');
            Ki = fscanf(mySerial, '%f');
            fprintf('The current controller is using Kp = %.3f and Ki = %.3f.\n', Kp, Ki); 
        case 'i'                        % Set position gains
            
            %WIP
            kp_pos = input('');
            fprintf(mySerial, '%f\n', kp_pos);
            ki_pos = input('');
            fprintf(mySerial, '%f\n', ki_pos);
            kd_pos = input('');
            fprintf(mySerial, '%f\n', kd_pos);
            
        case 'j'                        % Get position gains
            
            %WIP
            
        case 'k'                        % Test current control
            nsamples = fscanf(mySerial,'%d');       % first get the number of samples being sent
            data = zeros(nsamples,2);               % two values per sample:  ref and actual
            for i=1:nsamples
                data(i,:) = fscanf(mySerial,'%d %d'); % read in data from PIC32; assume ints, in mA
                times(i) = (i-1)*0.2;                 % 0.2 ms between samples
            end
            if nsamples > 1						        
                stairs(times,data(:,1:2));            % plot the reference and actual
            else
                fprintf('Only 1 sample received\n')
                disp(data);
            end
            % compute the average error
            score = mean(abs(data(:,1)-data(:,2)));
            fprintf('\nAverage error: %5.1f mA\n',score);
            title(sprintf('Average error: %5.1f mA',score));
            ylabel('Current (mA)');
            xlabel('Time (ms)');   
        case 'l'                        % Go to angle (deg)
                       
            %WIP
        
        case 'q'
            has_quit = true;             % Exit client
        case 'p'                         % Unpower the motor
            fprintf('PIC32 reset to IDLE mode.\n');
        case 'r'                         % Get mode
            mode = fscanf(mySerial, '%s');
            fprintf('PIC32 currently in %s mode.\n', mode);
        otherwise
            fprintf('Invalid Selection %c\n', selection);
    end
end

end
