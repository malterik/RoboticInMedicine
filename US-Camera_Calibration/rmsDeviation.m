function rmsd = rmsDeviation(pObs, meanPt)
%RMSDEVIATION Computes RMS deviation in the observations from their mean.
%   PROBEPOSES = RMSDEVIATION(POBS, MEANPT) POBS contains all the observed 
%   measurements and MEANPT is their mean.

numRep = size(pObs,2);
rmsd = sqrt(mean(sum((pObs - repmat(meanPt, 1, numRep)).^2)));