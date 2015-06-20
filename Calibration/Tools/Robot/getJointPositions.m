function [ joints, message ] = getJointPositions(socket, inStream, outStream)
%GETJOINTPOSITIONS Summary of this function goes her
%   Detailed explanation goes here
    import java.net.*;
    import java.io.*;
    
    cmd = 'GetPositionJoints';
    message = getAnswerFromServer(socket, inStream, outStream, cmd);
    messageSplit = strsplit(message,' ');
    joints = str2double(messageSplit);
end

