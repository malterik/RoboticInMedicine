function [] = clearInBuffer( inStream )
%CLEARINBUFFER Summary of this function goes here
%   Detailed explanation goes here
    while (inStream.ready())
       inStream.readLine(); 
    end
end

