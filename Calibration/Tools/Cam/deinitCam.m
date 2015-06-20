function [] = deinitCam(socket, inStream, outStream)
%DEINITCAM Summary of this function goes here
%   Detailed explanation goes here
    import java.net.*;
    import java.io.*;    

    outStream.println('Quit');
    socket.close();
    inStream.close();
    outStream.close();
end

