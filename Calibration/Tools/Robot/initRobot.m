function [ socket, inStream, outStream, message ] = initRobot( ip, port, timeout )
%INITROBOT Summary of this function goes here
%   Detailed explanation goes her 
    import java.net.*;
    import java.io.*;

    % connect
    socket = Socket(ip, port);
    socket.setSoTimeout(timeout);
    inStream = BufferedReader(InputStreamReader(socket.getInputStream));
    outStream = PrintWriter(socket.getOutputStream,true);    
    
    % authentication
    cmd = 'Hello Robot';
    message = getAnswerFromServer(socket, inStream, outStream, cmd);
end

