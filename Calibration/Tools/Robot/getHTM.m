function [ HTM, message ] = getHTM(socket, inStream, outStream)
%GETHTM Summary of this function goes here
%   Detailed explanation goes here
    import java.net.*;
    import java.io.*;
    
    cmd = 'GetPositionHomRowWise';
    message = getAnswerFromServer(socket, inStream, outStream, cmd);
    messageSplit = strsplit(message,' ');
    HTM = [reshape(str2double(messageSplit), [4, 3])'; 0 0 0 1];
end

