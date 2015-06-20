function [ outAngles ] = adjustAngles( defaultAngles, inAngles )
%ADJUSTANGLES Summary of this function goes here
%   Detailed explanation goes here

    outAngles = zeros(6,1);

    for i=1:length(inAngles);
       angle = inAngles(i);
       if (inAngles(i) > 0)
           otherAngle = inAngles(i) - 360;
       else
           otherAngle = inAngles(i) + 360;
       end
       
       if (abs(defaultAngles(i) - angle) < abs(abs(defaultAngles(i) - otherAngle)))
           outAngles(i) = angle;
       else
           outAngles(i) = otherAngle;
       end
    end
end

