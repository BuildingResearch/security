% Input file: Contains recorded earthquake data.
% Output file: Recorded data re-formatted.
%
function [dt,data] = init_earthquake_data(input_filename)
%
% open input file
fin = fopen(input_filename,'r');
%
% initialize following variables:
% flag for plotting numeric earthquake data
flag = 0;
% sample time
dt = 0;
% earthquake data
%data = zeros(1,5);
% index
i = 1;
%
while 1
    % read line from input file into string buffer
    strbuf = fgetl(fin);
    %
    % stop loop if finished reading input file
    if ~ischar(strbuf)
        break
    end
    %
    % if flagged, copy earthquake data to output variable 'data'
    if (flag == 1)
        data_buf = str2num(strbuf);
        % calc size in case less than 5 data points
        n = size(data_buf);
        n = n(2);
        % if less than 5 columns of data, pad data buffer with zeros
        if n < 5
            j = n + 1;
            while j <= 5
                data_buf(j) = 0;
                j = j + 1;
            end
        end
        data(i,:) = data_buf;
        i = i + 1;
    end    
    %
    % search for sample time
    if (flag == 0)
        loc = findstr(strbuf,'DT');
        %
        % if sample time found, save to output variable 'dt'
        if (loc > 0)
            % sample time

            % new PEER earthquake data; older files have different header
            % format
            dt= str2num(strbuf(18:26));
            
            % flag to start collecting earthquake data
            flag = 1;
        end    
    end
end
%
% close input file
fclose(fin);

