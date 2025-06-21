% Specify the file to monitor
fileToMonitor = 'test.txt';

% Initialize a variable to keep track of the number of lines
previousNumLines = 0;

disp('Monitoring the file for new lines...');

while true
    fileContent = fileread(fileToMonitor);
    
    % Count the number of lines in the file
    lines = textscan(fileContent, '%s', 'Delimiter', '\n');
    currentNumLines = numel(lines{1});
    
    % Check if a new line has been added
    if currentNumLines > previousNumLines
        % New lines detected
        disp('New line(s) detected!');
        
        % Process the new lines
        newLines = lines{1}(previousNumLines + 1:currentNumLines);
        disp('New line content:');
        disp(newLines);
        
        % Update the previous line count
        previousNumLines = currentNumLines;
    end
  
    % Pause for a short interval to avoid excessive CPU usage
    pause(1);
end