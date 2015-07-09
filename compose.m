load('Calibration/Data/camHTMsComp.mat');
load('Calibration/Data/robHTMsComp.mat');


for i = 1:8;
    tempRob(:,:,i) = composedRobHTMs(:,:,i+30);
    composedRobHTMs(:,:,i+30) = composedCamHTMs(:,:,i+30);
    composedCamHTMs(:,:,i+30) = tempRob(:,:,i);
end

robHTMs = composedRobHTMs;
camHTMs = composedCamHTMs;
save('Calibration/Data/robHTMsComp.mat', 'robHTMs');
save('Calibration/Data/camHTMsComp.mat', 'camHTMs');