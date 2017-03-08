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
    fprintf('     q: Quit\n');
    fprintf('     a: Read current sensor (ADC counts)\n');
    fprintf('     b: Read current sensor (mA)\n');
    fprintf('     c: Read encoder (counts)\n');
    fprintf('     d: Read encoder (deg)\n');
    fprintf('     e: Reset encoder\n');
    fprintf('     f: Set PWM (-100 to 100)\n');
    fprintf('     p: Unpower the motor\n');
    fprintf('     r: Get mode\n');
   
        % read the user's choice
    selection = input('\nENTER COMMAND: ', 's');
     
    % send the command to the PIC32
    fprintf(mySerial,'%c\n',selection);
    
    % take the appropriate action
    switch selection     
        case 'q'
            has_quit = true;             % exit client
        case 'a'                         % Read current sensor (ADC counts)
            adc_clkticks = fscanf(mySerial, '%d');
            fprintf('ADC clock counts: %d\n', adc_clkticks);
        case 'b'                         % Read current sensor (mA)
            adc_estamps = fscanf(mySerial, '%f');
            fprintf('ADC-derived current: %f mA\n', adc_estamps); 
        case 'c'                         % Read encoder counts
            counts = fscanf(mySerial, '%d');
            fprintf('The motor angle is %d counts.\n', counts);
        case 'd'                         % Read encoder deg
            deg_100x = fscanf(mySerial, '%d');
            fprintf('The motor angle is %0.1f degrees\n', deg_100x/100.0);
        case 'e'                         % Reset encoder
        case 'f'                         % Set PWM (-100 to 100)
            dutycycle = input('Enter desired duty cycle (-100 to 100): ');
            fprintf(mySerial, '%d', dutycycle);
        case 'p'                         % Unpower the motor
        case 'r'                         % Get mode
            mode = fscanf(mySerial, '%s');
            fprintf('PIC32 mode: %s\n', mode);
        otherwise
            fprintf('Invalid Selection %c\n', selection);
    end
end

end
