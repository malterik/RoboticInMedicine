function [] = waitForCompletion(socket, inStream, outStream, displayMessage, pollTime)
%WAITFORROBOT Summary of this function goes here
%   Detailed explanation goes here
    import java.net.*;
    import java.io.*;
    
     while (1)
        cmd = 'GetQueueLength';
        message = getAnswerFromServer(socket, inStream, outStream, cmd);
        queueLength = str2double(message);
        disp(sprintf('\t%s', displayMessage));
        if (queueLength == 0)
           break;
        end
       pause(pollTime);
    end
end

