function [Y , Z] = calculateposlidbeams(fileName,index_gate,NumberOfBeams)

fid = fopen(fileName, 'r');

% 1. Skip the first 24 lines to reach the beam data 
for i = 1:24
    fgetl(fid);
end

% 4 beam pulsed
if NumberOfBeams == 4

    % 2. Initialize storage
    azimuth = zeros(NumberOfBeams, 1);
    elevation = zeros(NumberOfBeams, 1);
    range_gates_matrix = zeros(NumberOfBeams, 10); % Your file shows 10 range gates per beam [cite: 4]
    
    % 3. Read the 4 beam lines 
    for i = 1:NumberOfBeams
        tline = fgetl(fid); % Read the entire line as a string
        
        % str2num is robust: it ignores the extra spaces and converts the string to an array
        values = str2num(tline); 
        
        if ~isempty(values)
            azimuth(i) = values(1);      % First value is Azimuth 
            elevation(i) = values(2);    % Second value is Elevation 
            range_gates_matrix(i, :) = values(3:end); % Remaining 10 are Range Gates 
        end
    end
    
    fclose(fid);
    
    range_gate = range_gates_matrix(:,index_gate);
    

elseif NumberOfBeams == 50

        % 2. Initialize storage
    azimuth = zeros(NumberOfBeams, 1);
    elevation = zeros(NumberOfBeams, 1);
    range_gate = zeros(NumberOfBeams, 1); 
    
    % 3. Read the 4 beam lines 
    for i = 1:NumberOfBeams
        tline = fgetl(fid); % Read the entire line as a string
        
        % str2num is robust: it ignores the extra spaces and converts the string to an array
        values = str2num(tline); 
        
        if ~isempty(values)
            azimuth(i) = values(1);      % First value is Azimuth 
            elevation(i) = values(2);    % Second value is Elevation 
            range_gate(i, :) = values(3); % third value is range gate 
        end
    end
    
    fclose(fid);
        
end

    Y = (range_gate.*cosd(azimuth).*sind(elevation));
    Z = range_gate.*sind(elevation);

end