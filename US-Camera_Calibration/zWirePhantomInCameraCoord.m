<<<<<<< HEAD
function [p, TAll] = zWirePhantomInCameraCoord(camServerIP, numRepition)
%ZWIREPHANTOMINCAMERACOORD Connects to the "CambarServer"
%   ZWIREPHANTOMINCAMERACOORD(CAMSERVERIP)
%   Connects to the "CambarServer" with IP address of CAMSERVERIP at port
%   3000.
%
%   ZWIREPHANTOMINCAMERACOORD(IPSTRVAR,NUMREPITION) Connects to the
%   "CambarServer" with with IP address of CAMSERVERIP at port
%   3000. It repeats the measurement NUMREPITION times (Default: 5).
%
%   Returns the coordinates of the tip of stylus-locator in 'p' and the
%   transformations of the stylus-locator in 'TAll' relative to the camera 
%   coordinate system. 
% 
%   This function assumes that the Cambar Tracking has been trained to
%   track the 'stylus' locator. The 'stylus' is a custom 3D-printed object
%   with four reflective markers located on it. 
%
%   The CambarServer should be able to find the 'stylus'-locator file at 
%   'C:\locators\' in the PC where CambarServer is running.
%   
%   Modified 22th May 2015
%   Omer Rajput - MTEC, TUHH.

% handle input arguments
if nargin == 1
    numRepition = 5;
elseif nargin > 2 && nargin == 0
    error('err:zWirePhantomInCameraCoord:wrongArguments', ...
        ['There was an unsupported number of input arguments\n',...
         'Usage: zWirePhantomInCameraCoord(''134.28.45.63'')', ...
         '   or: zWirePhantomInCameraCoord(''134.28.45.63'', 5)' ]);
end

camServerPort = 3000;

% Open the TCP/IP-Connection
disp('Connecting to Cambar Server')
jTcpObj = jtcp('request', camServerIP, camServerPort,'serialize',false);
pause(0.1);
mssg = char(jtcp('read',jTcpObj)); disp(mssg)

% Send the keyword mtec to authorize the client
jtcp('write',jTcpObj,int8('mtec')); pause(0.1);
mssg = char(jtcp('read',jTcpObj)); disp(mssg);

disp('Loading Stylus locator')
% Load locator definition file stylus.xml from the config folder
% The locator must be located on the camera-pc in the folder
% C:\locators\
jtcp('write',jTcpObj,int8('LoadLocator stylus')); pause(0.1);
mssg = char(jtcp('read',jTcpObj)); disp(mssg);

jtcp('write',jTcpObj,int8('LoadLocator stylus')); pause(0.1);
mssg = char(jtcp('read',jTcpObj)); disp(mssg);

disp('Starting the measurements...');
disp(['Measure the four points of the phantom ', num2str(numRepition),...
    ' times in the same order.']);
disp(['Make sure that you have correctly placed the tip of the stylus',...
     ' at the location you want to measure.']);
disp('Press any key when you are ready for the first measurement...');
% A loop to aquire data continously
N = 4; % for 4 wire-end-points on the z-shape.
figure;hold on;pause;
p = zeros(4,N*numRepition);
TAll = zeros(16,N*numRepition);
for rep = 1:numRepition
    i=1;
    while (i<=N)
        % Send the command word to get the locator position. The locator has to
        % be loaded before.
        [T,timestamp] = GetLocatorTransformMatrix(t, 'stylus');
        if (sum(T(:))~=1)
            absPtNum = i+(rep-1)*N;
            TAll(:,absPtNum) = T(:);
            % transform marker center point in camera coordinates
            p(:,absPtNum) = T * [0,0,0,1]';
            disp(['Timestamp: ' num2str(timestamp) 'rep: ' num2str(rep) ', p' num2str(i) ': ' num2str(p(:,absPtNum)')])
            % plot the marker position
            plot3(p(1,absPtNum),p(2,absPtNum),p(3,absPtNum),'ko');
            title(['rep: ' num2str(rep) ', measurements so far ' num2str(absPtNum)]);
            % wait 0.001 seconds
            disp('Press any key to continue to the next measurement...')
            pause;
            i=i+1;
        else
            disp('All zero T received. Try the same measurement again.')
        end
    end
end
hold off;

prompt = 'Do you want to save the measured data for later use? Y/N [N]: ';
str = input(prompt,'s');
if strcmpi(str,'y')
    mkdir('./','ultrasoundImagesAndPoses')
    save('ultrasoundImagesAndPoses/zWireEndPoints.mw','p','TAll');
=======
function [p, TAll] = zWirePhantomInCameraCoord(camServerIP, numRepition)
%ZWIREPHANTOMINCAMERACOORD Connects to the "CambarServer"
%   ZWIREPHANTOMINCAMERACOORD(CAMSERVERIP)
%   Connects to the "CambarServer" with IP address of CAMSERVERIP at port
%   3000.
%
%   ZWIREPHANTOMINCAMERACOORD(IPSTRVAR,NUMREPITION) Connects to the
%   "CambarServer" with with IP address of CAMSERVERIP at port
%   3000. It repeats the measurement NUMREPITION times (Default: 5).
%
%   Returns the coordinates of the tip of stylus-locator in 'p' and the
%   transformations of the stylus-locator in 'TAll' relative to the camera 
%   coordinate system. 
% 
%   This function assumes that the Cambar Tracking has been trained to
%   track the 'stylus' locator. The 'stylus' is a custom 3D-printed object
%   with four reflective markers located on it. 
%
%   The CambarServer should be able to find the 'stylus'-locator file at 
%   'C:\locators\' in the PC where CambarServer is running.
%   
%   Modified 22th May 2015
%   Omer Rajput - MTEC, TUHH.

% handle input arguments
if nargin == 1
    numRepition = 5;
elseif nargin > 2 && nargin == 0
    error('err:zWirePhantomInCameraCoord:wrongArguments', ...
        ['There was an unsupported number of input arguments\n',...
         'Usage: zWirePhantomInCameraCoord(''134.28.45.63'')', ...
         '   or: zWirePhantomInCameraCoord(''134.28.45.63'', 5)' ]);
end

camServerPort = 3000;

% Open the TCP/IP-Connection
disp('Connecting to Cambar Server')
jTcpObj = jtcp('request', camServerIP, camServerPort,'serialize',false);
pause(0.1);
mssg = char(jtcp('read',jTcpObj)); disp(mssg)

% Send the keyword mtec to authorize the client
jtcp('write',jTcpObj,int8('mtec')); pause(0.1);
mssg = char(jtcp('read',jTcpObj)); disp(mssg);

disp('Loading Stylus locator')
% Load locator definition file stylus.xml from the config folder
% The locator must be located on the camera-pc in the folder
% C:\locators\
jtcp('write',jTcpObj,int8('LoadLocator stylus')); pause(0.1);
mssg = char(jtcp('read',jTcpObj)); disp(mssg);

jtcp('write',jTcpObj,int8('LoadLocator stylus')); pause(0.1);
mssg = char(jtcp('read',jTcpObj)); disp(mssg);

disp('Starting the measurements...');
disp(['Make sure that you have correctly placed the tip of the stylus',...
     ' at the location you want to measure.']);
disp('Press any key when you are ready for the first measurement...');
% A loop to aquire data continously
N = 4; % for 4 wire-end-points on the z-shape.
figure;hold on;pause;
p = zeros(4,N*numRepition);
TAll = zeros(16,N*numRepition);
for rep = 1:numRepition
    i=1;
    while (i<=N)
        % Send the command word to get the locator position. The locator has to
        % be loaded before.
        [T,timestamp] = GetLocatorTransformMatrix(t, 'stylus');
        if (sum(T(:))~=1)
            absPtNum = i+(rep-1)*N;
            TAll(:,absPtNum) = T(:);
            % transform marker center point in camera coordinates
            p(:,absPtNum) = T * [0,0,0,1]';
            disp(['Timestamp: ' num2str(timestamp) 'rep: ' num2str(rep) ', p' num2str(i) ': ' num2str(p(:,absPtNum)')])
            % plot the marker position
            plot3(p(1,absPtNum),p(2,absPtNum),p(3,absPtNum),'ko');
            title(['rep: ' num2str(rep) ', measurements so far ' num2str(absPtNum)]);
            % wait 0.001 seconds
            disp('Press any key to continue to the next measurement...')
            pause;
            i=i+1;
        else
            disp('All zero T received. Try the same measurement again.')
        end
    end
end
hold off;

prompt = 'Do you want to save the measured data for later use? Y/N [N]: ';
str = input(prompt,'s');
if strcmpi(str,'y')
    mkdir('./','ultrasoundImagesAndPoses')
    save('ultrasoundImagesAndPoses/zWireEndPoints.mw','p','TAll');
>>>>>>> origin/master
end