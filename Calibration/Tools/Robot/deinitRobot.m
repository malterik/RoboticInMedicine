function [] = deinitRobot(socket, inStream, outStream)
%DEINITROBOT Summary of this function goes here
%   Detailed explanation goes here
    import java.net.*;
    import java.io.*;    

    socket.close();
    inStream.close();
    outStream.close();
end

