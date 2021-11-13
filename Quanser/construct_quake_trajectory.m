% Constructs a trajectory from a matrix of position data given the
% the sample time.
% Input:    X    --> r X c matrix, where r = # of rows and c = # of columns
% Ouput:    data --> single dimension array with matrix X data.
%           time --> single dimesion array with time corresponding to
%                    position.
% 
function [time,pos] = construct_quake_trajectory(X,dt)
% calc # of rows and columns
r = max(size(X));
c = min(size(X));
%
% construct [time,data] array
n = 1;
for i = 1:r
    for j = 1:c
        pos(n) = X(i,j);
        time(n) = (n-1)*dt;
        n = n+1;
    end
end