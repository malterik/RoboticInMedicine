save('Calibration/Data/robHTMsPres.mat', 'robHTMs');
save('Calibration/Data/camHTMsPres.mat', 'camHTMs');
save('Calibration/Data/robHTMsManualPres.mat', 'robPoses');
save('Calibration/Data/camHTMsManualPres.mat', 'camPoses');

composedRobHTMs = robHTMs;
composedCamHTMs = camHTMs;

for i = 1:size(robPoses,3);
    composedRobHTMs(:,:,30+i) = camPoses(:,:,i);
    composedCamHTMs(:,:,30+i) = robPoses(:,:,i);
end

save('Calibration/Data/robHTMsComp.mat', 'composedRobHTMs');
save('Calibration/Data/camHTMsComp.mat', 'composedCamHTMs');