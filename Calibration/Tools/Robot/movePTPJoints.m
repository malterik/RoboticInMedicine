function [message] = movePTPJoints(socket, inStream, outStream, joints)
%MOVEPTPJOINTS Summary of this function goes here
%   Detailed explanation goes here
    import java.net.*;
    import java.io.*;
    
    cmd = sprintf('MovePTPJoints %s', sprintf('%f ', joints));
    message = getAnswerFromServer(socket, inStream, outStream, cmd);
end

