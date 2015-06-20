function [ outHTM, HTMJoints] = createNearbyHTM( inJoints, inHTM, maxAngle, maxTranslation )
%CREATENEARBYHTM Summary of this function goes here
%   Detailed explanation goes here
    
    % rotational part
    axis = rand(3,1)-0.5;
    angle = maxAngle.*randn(1);        
    rot = vrrotvec2mat([axis' angle]);
    rot = rot(1:3,1:3);   

    % translational part
    minTranslation = -maxTranslation;
    translationDirections = rand(3,1);
    translationNorm = (maxTranslation-minTranslation)*rand(1,1) + minTranslation;
    translation = translationNorm * (translationDirections/norm(translationDirections));

    % compose rotation and translation to HTM
    H = [rot translation; 0 0 0 1;];
    
    % calculate new calibration pose
    outHTM = H*inHTM;
    
    % translate to joints using inverse kinematics
    [jointPositions,configuration] = inverseKinematicsAndPlanningUR5(outHTM);
    jointPositions = jointPositions*180/pi;
    
    for i = 1:size(jointPositions,2);
        jointPositions(:,i) = adjustAngles( inJoints, jointPositions(:,i));
    end    
    
    minValue = realmax;
    bestJoints = 1;
    for i=1:size(jointPositions,2)
        sumOfAngles = sum(abs((inJoints-jointPositions(:,i))));
        if (sumOfAngles < minValue);
           minValue = sumOfAngles;
           bestJoints = i; 
        end
    end
    
    HTMJoints = jointPositions(:,bestJoints);
    

end

