function [T, timestamp, isVisible, message] = getLocatorTransformMatrix(socket, inStream, outStream, locatorName)
% GetLocatorTransformMatrix  Gets the transform matrix of the locator
% 'name'.
% TCP/IP object associated with the CambarServer needs to be provided as
% well.
    import java.net.*;
    import java.io.*;

    cmd = sprintf('GetLocatorPosition %s', locatorName);
    message = getAnswerFromServer(socket, inStream, outStream, cmd);
    messageSplit = strsplit(message,' ');
    numbers = str2double(messageSplit);
    % Format:
    % 1. timestamp
    % 2. visible-flag
    % 3. 4x4 position ,matrix of the locator:
    %   R00, R01, R02, X, R10, R11, R12, Y, R20, R21, R22, Z, 0, 0, 0, 1
    T = reshape(numbers(3:end), 4,4)';
    timestamp = numbers(1);
    zeroMatrix = [0 0 0 0; 0 0 0 0; 0 0 0 0; 0 0 0 1];
    isVisible = ~isequal(T,zeroMatrix);
end