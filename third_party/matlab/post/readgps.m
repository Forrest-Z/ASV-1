function gps = readgps(filename)

delimiter = ',';
startRow = 2;

%% Read columns of data as text:
% For more information, see the TEXTSCAN documentation.
formatSpec = '%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%q%[^\n\r]';

%% Open the text file.
fileID = fopen(filename,'r');

%% Read columns of data according to the format.
% This call is based on the structure of the file used to generate this
% code. If an error occurs for a different file, try regenerating the code
% from the Import Tool.
dataArray = textscan(fileID, formatSpec, 'Delimiter', delimiter, 'TextType', 'string', 'HeaderLines' ,startRow-1, 'ReturnOnError', false, 'EndOfLine', '\r\n');

%% Close the text file.
fclose(fileID);

%% Convert the contents of columns containing numeric text to numbers.
% Replace non-numeric text with NaN.
raw = repmat({''},length(dataArray{1}),length(dataArray)-1);
for col=1:length(dataArray)-1
    raw(1:length(dataArray{col}),col) = mat2cell(dataArray{col}, ones(length(dataArray{col}), 1));
end
numericData = NaN(size(dataArray{1},1),size(dataArray,2));

for col=[1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,18,19,20]
    % Converts text in the input cell array to numbers. Replaced non-numeric
    % text with NaN.
    rawData = dataArray{col};
    for row=1:size(rawData, 1)
        % Create a regular expression to detect and remove non-numeric prefixes and
        % suffixes.
        regexstr = '(?<prefix>.*?)(?<numbers>([-]*(\d+[\,]*)+[\.]{0,1}\d*[eEdD]{0,1}[-+]*\d*[i]{0,1})|([-]*(\d+[\,]*)*[\.]{1,1}\d+[eEdD]{0,1}[-+]*\d*[i]{0,1}))(?<suffix>.*)';
        try
            result = regexp(rawData(row), regexstr, 'names');
            numbers = result.numbers;
            
            % Detected commas in non-thousand locations.
            invalidThousandsSeparator = false;
            if numbers.contains(',')
                thousandsRegExp = '^\d+?(\,\d{3})*\.{0,1}\d*$';
                if isempty(regexp(numbers, thousandsRegExp, 'once'))
                    numbers = NaN;
                    invalidThousandsSeparator = true;
                end
            end
            % Convert numeric text to numbers.
            if ~invalidThousandsSeparator
                numbers = textscan(char(strrep(numbers, ',', '')), '%f');
                numericData(row, col) = numbers{1};
                raw{row, col} = numbers{1};
            end
        catch
            raw{row, col} = rawData{row};
        end
    end
end


%% Split data into numeric and string columns.
rawNumericColumns = raw(:, [1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,18,19,20]);
rawStringColumns = string(raw(:, 17));


%% Replace non-numeric cells with NaN
R = cellfun(@(x) ~isnumeric(x) && ~islogical(x),rawNumericColumns); % Find non-numeric cells
rawNumericColumns(R) = {NaN}; % Replace non-numeric cells

%% Make sure any text containing <undefined> is properly converted to an <undefined> categorical
idx = (rawStringColumns(:, 1) == "<undefined>");
rawStringColumns(idx, 1) = "";

%% Create output variable
gps.ID = cell2mat(rawNumericColumns(:, 1));
gps.DATETIME = cell2mat(rawNumericColumns(:, 2));
gps.gps_week = cell2mat(rawNumericColumns(:, 3));
gps.gps_time = cell2mat(rawNumericColumns(:, 4));
gps.heading = cell2mat(rawNumericColumns(:, 5));
gps.pitch = cell2mat(rawNumericColumns(:, 6));
gps.roll = cell2mat(rawNumericColumns(:, 7));
gps.latitude = cell2mat(rawNumericColumns(:, 8));
gps.longitude = cell2mat(rawNumericColumns(:, 9));
gps.altitude = cell2mat(rawNumericColumns(:, 10));
gps.Ve = cell2mat(rawNumericColumns(:, 11));
gps.Vn = cell2mat(rawNumericColumns(:, 12));
gps.Vu = cell2mat(rawNumericColumns(:, 13));
gps.base_line = cell2mat(rawNumericColumns(:, 14));
gps.NSV1 = cell2mat(rawNumericColumns(:, 15));
gps.NSV2 = cell2mat(rawNumericColumns(:, 16));
gps.gpsstatus = categorical(rawStringColumns(:, 1));
gps.gpscheck = cell2mat(rawNumericColumns(:, 17));
gps.UTM_x = cell2mat(rawNumericColumns(:, 18));
gps.UTM_y = cell2mat(rawNumericColumns(:, 19));

%% Clear temporary variables
clearvars filename delimiter startRow formatSpec fileID dataArray ans raw col numericData rawData row regexstr result numbers invalidThousandsSeparator thousandsRegExp rawNumericColumns rawStringColumns R idx;


end