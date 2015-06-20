%%%%%%%%%%%%%%%%%%% Sven-Thomas Antoni, MTEC TUHH, 2014 %%%%%%%%%%%%%%%%%%%
%
% Calculates inverse kinematics for all available configurations at once
% and choses the best solution depending on the actual joint position
%
%--------------------------------------------------------------------------
% Input:
% endPose:          4x4 double; pose of the endeffector for which the
%                   inverse kinematics should be done
%--------------------------------------------------------------------------
% Output:
% jointPositions:   1x6 double; calculated joint positions to given pose
%--------------------------------------------------------------------------
% Global:
% debug:            1x1 boolean; true if debugmode should be enabled, false 
%                   or unset else (unused atm)
% ARM/ELBOW/WRIST:  1x1 or 1x2 double; determins which solutions should be
%                   calculated, the different configurations are identified
%                   by +1 and -1, other values are not supported. If unset
%                   [-1,+1] will be used...
% connectingTime:   1x1 tic-handle; time the input connection to the robot
%                   was established (unset possible)
% inputStream:      1x1 java.net.SocketOutputStream; used to determine if
%                   connection to robot is available: unset: offline, set:
%                   online
% timeCrtical:      1x1 boolean; when set readAndInterpret will check the
%                   actuallity of the aquired data, can be unset and will
%                   set if neccessary
% DH:               1x1 struct; the DH parameters of the UR5, will be set
% reachable:        1xnumSolutions boolean; determines whether a solution
%                   is reachable or not... will be set
% choosePathHandle: function handle; handle to the function that should be
%                   used to determine the best solution, (unset possible)
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [jointPositions,configuration] = inverseKinematicsAndPlanningUR5(endPose)

global ARM ELBOW WRIST inputStream timeCritical DH reachable ...
    choosePathHandle debug

% catch unset global variables
% if not set otherwise allow all possible solutions
configStr={'ARM','ELBOW','WRIST'};
for i=1:length(configStr)
    if eval(['isempty(',configStr{i},')'])
        eval([configStr{i},'=[-1,+1];']);
    elseif eval(['length(',configStr{i},')==2'])
        eval([configStr{i},'=reshape(',configStr{i},',1,2);']);
    end
end

%catch invalid input
if nargin~=1
    error('err:inverseKinematicsAndPlanningsUR5:wrongInput',...
        'Wrong number of input arguments!')
elseif ~all(size(endPose)==[4,4])
    error('err:inverseKinematicsAndPlanningsUR5:wrongInput',...
        'Given pose is not a valid pose!')
end

% from available configurations determine the maximal number of possible
% solutions and predefine theta and bool-vector for reachables accordingly
numSolutions=length(ARM)*length(ELBOW)*length(WRIST);
theta=zeros(6,numSolutions);
reachable=ones(numSolutions,1);

% identify configurations (not used atm, maybe for debug or information in
% the future)
temp=repmat({ones(length(WRIST)*length(ELBOW),1)},1,length(ARM));
config(1,:)=ARM*blkdiag(temp{:})';
temp=repmat({ones(length(ELBOW),1)},1,length(WRIST));
temp=repmat(blkdiag(temp{:}),length(ARM),1);
config(2,:)=WRIST*temp';
config(3,:)=repmat(ELBOW,1,length(WRIST)*length(ARM));

% DH parameters of UR5
DH.a=[0,-0.4250,-0.39225,0,0,0];
DH.d=[0.089159, 0 , 0 , 0.10915, 0.09465, 0.0823];
DH.alpha=[pi/2, 0,0, pi/2, -pi/2, 0];

%center of Hand Rotation
x=endPose(1:3,1:3)*[1,0,0]';
y=endPose(1:3,1:3)*[0,1,0]';
z=endPose(1:3,1:3)*[0,0,1]';

% position of hand
posHand=endPose(1:3,4)-DH.d(6)*z;

%% theta1
% triangle O5 and O0, distance O5 to the 3R-Arm is fixed (=d4)
theta(1,1:numSolutions)=reshape(ones(length(ELBOW)*length(WRIST),1)*(atan2(posHand(2),posHand(1))+ARM*(acos(DH.d(4)/sqrt(posHand(2)^2+posHand(1)^2)))+pi/2),1,numSolutions);

%% theta 5
% triangle O5, O6 and 06-h (thus right triangle), O5<->O6 is d6 and
% O6<->O6-h is h, whereby h is the height and such
% h=abs(endPose(1,4)*sin(theta(1))+endPose(2,4)*cos(theta(1)))-d4
% then use sin(pi/2-alpha)=cos(alpha)=b/c=h/d6: et voila!

% temp variable in order to calculate all possible solutions at once...
temp=repmat({ones(length(ELBOW),1)},1,length(WRIST)*length(ARM));

% depending on given ARM configuration choose theta1(s) used for
% calculation
choice=numSolutions/length(ARM)+ (length(ARM)-1)*linspace(0,1,length(ARM));

% calculate theta5
theta(5,1:numSolutions)=blkdiag(temp{:})*reshape(WRIST'*...
    acos((endPose(1,4)*sin(theta(1,choice))-endPose(2,4)*...
    cos(theta(1,choice))-DH.d(4))/DH.d(6)),length(ARM)*length(WRIST),1);

% handle theta5 being imaginary, which means that position is not reachable
if any(imag(theta(5,:)))~=0
    reachable=reachable +abs( ~imag(theta(5,:)))';
    theta=real(theta);
end

%% theta 6
% as theta(2) to theta(4) are parallel with theta(1) and theta(5)
% everything is known to calculate theta(6) (see paper)
theta(6,1:numSolutions)=atan2((-y(1)*sin(theta(1,:))+y(2)*...
    cos(theta(1,:)))./sin(theta(5,:)),-(-x(1)*sin(theta(1,:))+x(2)*...
    cos(theta(1,:)))./sin(theta(5,:)));

% if theta5 is zero, theta2,3,4,6 are parallel, in this case predefine
% theta 6
% if robot is connected predefine...
if isempty(inputStream)
    % ...with zero....
    theta(6,mod(theta(5,:),pi)==0)=0;
else % ... else as the actual joint position
    % try getting the actual joint position
    try
        saveTC=timeCritical;
        timeCritical=1;
        Data_temp=readAndInterpretUR5();    
        timeCritical=saveTC; 
        angle=Data_temp.ActualJointPositions(6);
    catch % if this fails predefine with zero again
        angle=0;
    end
    theta(6,mod(theta(5,:),pi)==0)=angle;
end

%% 3R-ARM (1) - theta2 and theta 3

% get shoulder position
posShoulder=[0;0;DH.d(1)];

% get wrist position(s)
% ... only for reachable positions
numReachable=countReachable;
temp=repmat({endPose},1,numReachable);
posWrist=blkdiag(temp{:})*directKinematicsAll(theta,[5:6])^-1;
temp=zeros(numReachable*4,1);
temp(2:4:end)=1;

posWristTemp=[getfield(diag(posWrist,3),{1:4:4*numReachable})';...
    getfield(diag(posWrist,2),{2:4:4*numReachable})';...
    getfield(diag(posWrist,1),{3:4:4*numReachable})';...
    zeros(1,numReachable)]-reshape(DH.d(4)*posWrist*temp,4,numReachable);

% if not reachable, posWrist is zero
posWrist=zeros(4,numSolutions);
posWrist(:,reachable~=0)=posWristTemp;

% calculate r, s, k and cos(phi)=-cos(theta3)=c
r=cos(theta(1,:)).*posWrist(1,:)+sin(theta(1,:)).*posWrist(2,:);
s=posWrist(3,:)-posShoulder(3);
k=sqrt(r.^2+s.^2);
c=-(k.^2-DH.a(2)^2-DH.a(3)^2)/(2*DH.a(2)*DH.a(3));

% catch impossible configurations
reachable(k>abs(sum(DH.a(2:3))))=0;
reachable(abs(c)>1)=0;

% catch near impossible configurations, those will only be used if no other
% configuration is possible
c(c>1)=1;
c(c<-1)=-1;
reachable(c>1 & c-1<0.003)=2;
reachable(c<-1 & abs(c)-1<0.003)=2;

% theta(3) and theta(2) according to 3R-Arm
theta(3,1:numSolutions)=atan2(reshape(getfield(ELBOW'*sqrt((1-c.^2)),...
    {1:length(ELBOW),1:length(ELBOW):numSolutions}),1,numSolutions),-c);
theta(2,1:numSolutions)=atan2(s,r)-atan2(DH.a(3)*sin(theta(3,:)),...
    DH.a(2)+DH.a(3)*cos(theta(3,:)));

%% 3R-ARM (2) - theta4
% for theta 4 center on the Wrist=O4  now: wrist=[0,0,0] and hand=posHand
numReachable=countReachable;
posHandTemp=reshape(directKinematicsAll(theta,1:3)^-1*...
    ([posHand;1]'*repmat(eye(4),1,numReachable))',4,numReachable);
posHand=zeros(4,numSolutions);
posHand(:,reachable~=0)=posHandTemp;

% triangle between wrist=center and the hand, calculate angle:
theta(4,:)=pi-atan2(posHand(1,:),posHand(2,:));
%keyboard
%% choose solution == planning

% only choose reachable thetas, should be advanced in future to only
% choose phisycally reachable thetas (with respect to the robot!)
% test whether there are thetas which are reachable without doubt
if ~isempty(reachable==1)
    preChoice=theta(:,reachable==1); %xx
else % if not also choose nearly impossible configurations
    preChoice=theta(:,reachable~=0); %xx
end

% only if robot is connected: try to choose an optimal solution
if ~isempty(inputStream)
    
    % if no method for choice is given use the default method which wil 
    % choosethe nearest solution in joint space
    if isempty(choosePathHandle)
        choosePathHandle=@(thetaChoice) chooseNearest(thetaChoice);
    end
    
    % choose solution
    [jointPositions,index]=choosePathHandle(preChoice);
    
    % only if requested
    if nargout>1
        % choose config
        index=getfield(find(reachable~=0,index),{index}); %xx
        temp=config(:,index);
        configuration(1:5,1)={'theta';'ARM';'WRIST';'ELBOW';'reachable'};
        configuration(1:5,2)=num2cell([index ; temp; reachable(index)]);
    end
else % if not connected: output all found solutions and the corresponding 
     % configuration
     jointPositions=preChoice;
     configuration(1:5,1)={'theta';'ARM';'WRIST';'ELBOW';'reachable'};
     configuration(1:5,2:sum(reachable~=0)+1)=...
        num2cell([find(reachable~=0)' ; config(:,reachable~=0);...
        reachable(reachable~=0)']);
end

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%% Technical Function %%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
% [pose] = directKinematicsAll(theta,interval)
%
% Does direct kinematics for a variable number >=1 of configurations at
% once, an intervall for which the direct kinematics should be calculated
% can ge forwarded
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [pose] = directKinematicsAll(theta,interval)

global DH reachable

numReachable=countReachable;
sizeBlock=4*numReachable;
A=eye(sizeBlock);
thetaReachable=theta(:,reachable~=0);

for i=interval
    temp=zeros(sizeBlock-1,1);
    temp(3:4:end)=DH.d(i);
    Tz=eye(sizeBlock)+diag(temp,1);
    temp=zeros(sizeBlock-3,1);
    temp(1:4:end)=DH.a(i);
    Tx=eye(sizeBlock)+diag(temp,3);
    
    Rz=eye(sizeBlock);
    Rx=eye(4);
    Rx(2:3,2:3)=[cos(DH.alpha(i)),-sin(DH.alpha(i));sin(DH.alpha(i)),...
        cos(DH.alpha(i))];
    temp=repmat({Rx},1,numReachable);
    Rx=blkdiag(temp{:});
    for j=0:numReachable-1
        Rz(4*j+[1:2],4*j+[1:2])=[cos(thetaReachable(i,j+1)),...
            -sin(thetaReachable(i,j+1));sin(thetaReachable(i,j+1)),...
            cos(thetaReachable(i,j+1))];
        
    end    
    A=A*Tz*Rz*Tx*Rx;
end
pose=A;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%% Technical Function %%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
% [numReachable] = countReachable()
%
% Determines how much possible solutions are reachable and will throw an
% error if no possible solutions exist anymore
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [numReachable] = countReachable()

global reachable

numReachable=sum(reachable~=0);

if numReachable==0
    error('err:inverseKinematicsAndPlanningUR5:noSolution',...
        'No solution could be found.')
end

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%% Technical Function %%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
% [choice] = chooseNearest(theta)
%
% Chooses the nearest solution to the actual joint position from various
% given possible thetas
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [choice,minDiffInd] = chooseNearest(theta)

global timeCritical

% get actual joint positions
saveTC=timeCritical;
timeCritical=1;
Data_temp=readAndInterpretUR5();
timeCritical=saveTC;
actualPosition=Data_temp.ActualJointPositions;

% find differences between actual and possible new joint positions
[difference,thetaOptimal]=diffJointPositions(theta,actualPosition');

% choose nearest joint position
[~,minDiffInd]=min(difference);
choice=thetaOptimal(:,minDiffInd);

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%% Technical Function %%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
% [difference,thetaOptimal] = diffJointPositions(thetaList,thetaAct)
%
% Calculates the difference between two joints positions in joint space.
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [difference,thetaOptimal] = diffJointPositions(thetaList,thetaAct)

% the joint positions possible on the ur5 are between -2pi an 2pi:
thetaList=sign(thetaList).*mod(abs(thetaList),2*pi);

% iterate over every theta in thetaList
for i=1:size(thetaList,2)
    % and compare to thetaAct
    diff=[];
    for j=1:size(thetaList,1)
        [diff(j),fac(j,i)]=min(abs(mod(thetaList(j,i)*[-1,1],2*pi)...
            .*[-1,1]-thetaAct(j)));
    end
    difference(i)=sum(diff);
end
fac=(fac-1)*2-1;
thetaOptimal=mod(thetaList.*fac,2*pi).*fac;
end
    
    
    
    
    
    
    
    
    
    
    

