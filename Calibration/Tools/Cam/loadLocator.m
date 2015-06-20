function [message] = loadLocator(socket, inStream, outStream, locatorName )
%LOADLOCATOR Summary of this function goes here
%   Detailed explanation goes here
    import java.net.*;
    import java.io.*;

    cmd = sprintf('LoadLocator %s', locatorName);
    message = getAnswerFromServer(socket, inStream, outStream, cmd);
    message = getAnswerFromServer(socket, inStream, outStream, cmd);
end

