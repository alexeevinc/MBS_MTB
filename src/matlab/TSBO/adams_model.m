%%%
% res = adams_modell([1,1,2]);

function y = adams_model(x)
file_output = 'output.txt';
file_input = 'input.txt';

% x_mod=[x, 0.1];
x_mod=[x];
output_ID = fopen(file_output, 'a');
fprintf(output_ID, '\n');
fprintf(output_ID, '%g ', x_mod);
fclose(output_ID);

% Count the number of lines in the file
lines = textscan(fileread(file_input), '%s', 'Delimiter', '\n');
previousNumLines = numel(lines{1});
disp(previousNumLines)

disp('Monitoring the INPUT file for new lines...');

while true
    % Check if a new line has been added
    
    lines = textscan(fileread(file_input), '%s', 'Delimiter', '\n');
    currentNumLines = numel(lines{1});

    if currentNumLines > previousNumLines
        disp('New line(s) detected!');
        % Initialize a variable to store the last line
        lastLine = '';
        % Read through the file line by line
        fID = fopen(file_input, 'r');
        while ~feof(fID)
            lastLine = fgetl(fID);  % Read the current line and store it
        end
     
        input_value=sscanf(lastLine, '%f');
        disp('break monitoring')
        break;
    end

    % Pause for a short interval to avoid excessive CPU usage
    pause(1);
end
disp(input_value)

y = input_value;

