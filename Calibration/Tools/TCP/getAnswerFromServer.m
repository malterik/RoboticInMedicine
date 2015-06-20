function [ line ] = getAnswerFromServer(socket, inStream, outStream, command)
%GETLINE Summary of this function goes here
%   Detailed explanation goes here
    clearInBuffer(inStream);
    outStream.println(command);
    line = char(inStream.readLine());
end

