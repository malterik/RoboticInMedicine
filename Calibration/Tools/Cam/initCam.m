function [ socket, inStream, outStream ] = initCam(ip, port, timeout )
%INITCAM Summary of this function goes here
%   Detailed explanation goes here
    import java.net.*;
    import java.io.*;
    
    % connect
    socket = Socket(ip, port);
    socket.setSoTimeout(timeout);
    inStream = BufferedReader(InputStreamReader(socket.getInputStream));
    outStream = PrintWriter(socket.getOutputStream,true);
    
    % authentication
    outStream.println('mtec');
end

