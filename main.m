%% Multiple Camera Calibration Toolbox - V1
% Laura Ribeiro, Tampere University, Finland
% (laura.goncalvesribeiro@tuni.fi)
%% Step 1
% Calibrates each camera independently using Matlab Camera Calibrator Application
% Calculates extrinsic parameters between camera N and camera 1
% Inputs: directory of calibration images, squareSize, boardSize
% Follow the naming convetion for calibration images: PositionXXX_CameraXX
%% Step 2
% Overall Optimization
% Options:
% Re-estimate all: [1, 1, 1 ,1];
% Fixed Intrinsics: [1, 1, 0 ,0];
% Fixed lens distortions: [0, 1, 0 ,0];
% Default: Fixed intrinsics.
%%  Load Calibration Images
clear all
addpath ('./resources/');
%% Replace with own values
directory1 = '.\TestSet\'; % Change to calibration image directory
squareSize = 20.0;  % in units of 'millimeters'
boardSize = [18,29];
%% Load Images
files1 = dir(directory1);
files1(1:2) = [];

imageFileNames = {files1(:).name};
cameras = [];

for i=1:length(imageFileNames)
    k = strfind(imageFileNames{i},'Camera')+6;
    cameras (i) = convertCharsToStrings(imageFileNames{i}(k:k+2));
    imageFileNames{i} = [directory1 imageFileNames{i}];
end

ucameras = unique(cameras);

for i=1: numel(ucameras)
    idx = find (cameras == ucameras(i));
    for n = 1: numel(idx)
        ifn{i,n}=char(imageFileNames{idx(n)});
    end
end
%% Calibrate Individual Cameras
numCams = size(ifn, 1);
numPos = size(ifn, 2);
% Generate world coordinates of the checkerboard keypoints
worldPoints = generateCheckerboardPoints(boardSize, squareSize);
ImagePoints = cell(1,numCams);
checkFound = cell(1,numCams);
imagesUsed = cell(1,numCams);
estimationErrors = cell(1,numCams);
fullImagesUsed = cell(1,numCams);
fullImagePoints = cell(1,numCams);

h = waitbar(0, ['Calibration Independently Camera: 1 of ' num2str(numCams)],...
                    'Name', 'Independent Camera Calibration');
for i=1:numCams % for each camera
    waitbar(i/numCams, h, ['Calibrating Independently Camera: ' num2str(i) ' of ' num2str(numCams)]);
    [ImagePoints{i}, ~ , checkFound{i}] = detectCheckerboardPoints(ifn(i,:)); % detect checkerboard for every position of camera i
    
    % Estimate Parameters Cam i
    [CameraParams{i}, imagesUsed{i}, estimationErrors{i}] = estimateCameraParameters(ImagePoints{i}, worldPoints, ...
        'EstimateSkew', false, 'EstimateTangentialDistortion', false, ...
        'NumRadialDistortionCoefficients', 3, 'WorldUnits', 'millimeters', ...
        'InitialIntrinsicMatrix', [], 'InitialRadialDistortion', []);
    
    % Keep track which images were used
    fullImagesUsed{i} = zeros(size(checkFound{i}));
    fullImagesUsed{i}(find(checkFound{i}==1)) = imagesUsed{i};
    
    ImagePointsAll{i} = NaN([size(ImagePoints{i},1), 2, size(ifn,1)]);
    ImagePointsAll{i}(:,:,fullImagesUsed{i}==1)= ImagePoints{i};
    
    %figure; showExtrinsics(CameraParams{i}, 'patternCentric'); title(['Extrinsics - Camera ', num2str(i)]);
end
delete(h);
%% Find positions seen by all cameras
used = all (cell2mat(fullImagesUsed),2); % All elements are non zero = All cameras see pattern
for cam = 1: numCams
    idx_allvisible{cam} = squeeze(used (checkFound{cam}==1));
    ImagePointsCommon{cam} = ImagePoints{cam}(:,:,idx_allvisible{cam});
end
%% Save Independent Calibration To File
formatOut = 'yyyy.mm.dd.HH.MM';
saveDirectory = [directory1,'Outputs'];
mkdir (saveDirectory);
save([saveDirectory,'\','IndependentCameraParameters.', datestr(now,formatOut),'.mat'],'CameraParams', 'idx_allvisible','ImagePointsAll','ImagePointsCommon', 'checkFound', 'fullImagesUsed', 'numCams', 'numPos', 'directory1');
%% Calculate relative R, t
for cam=2:numCams
    rotationVectors{cam} = NaN(numPos,3);
    rotationMatrices{cam} = NaN(numPos,3,3);
    translationVectors{cam} = NaN(numPos,3);
    for pos=1:numPos
        if (fullImagesUsed{cam}(pos) && fullImagesUsed{1}(pos))
            pos_converted = find(find(fullImagesUsed{1}==1)==pos);
            RTc1pn = RTtoTransform(CameraParams{1}.RotationMatrices(:, :, pos_converted),CameraParams{1}.TranslationVectors(pos_converted, :)');
            
            pos_converted = find(find(fullImagesUsed{cam}==1)==pos);
            RTcnpn= RTtoTransform(CameraParams{cam}.RotationMatrices(:, :, pos_converted), CameraParams{cam}.TranslationVectors(pos_converted, :)');
            RTcnc1 = inv(RTc1pn)*RTcnpn;
            
            rotationVectors{cam}(pos,:) = vision.internal.calibration.rodriguesMatrixToVector(RTcnc1(1:3, 1:3));
            rotationMatrices{cam}(pos,:,:) = RTcnc1(1:3, 1:3);
            translationVectors{cam}(pos,:) = RTcnc1(4, 1:3);
        end
    end
    r = nanmedian(rotationVectors{cam}, 1);
    rotationVectors_median{cam} = vision.internal.calibration.rodriguesVectorToMatrix(r);
    translationVectors_median{cam} = nanmedian(translationVectors{cam}, 1);
    RTcnc1median{cam} = RTtoTransform(rotationVectors_median{cam},translationVectors_median{cam});
end
%% Store Positions Pattern N relative to Camera 1
clear RTc1pn
tempR= CameraParams{1}.RotationMatrices(:,:,idx_allvisible{1});
tempt= CameraParams{1}.TranslationVectors(idx_allvisible{1},:);
for i=1:size(tempR,3)
    RTc1pn{i} = RTtoTransform(tempR(:,:,i),tempt(i,:));
end
%% Crete Calibration Struct
calibration.CameraParameters = CameraParams;
calibration.Extrinsics = RTcnc1median;
calibration.PatternPositions = RTc1pn;
calibration.ImagePointsCommon = ImagePointsCommon;
calibration.ImagePointsAll = ImagePointsAll;
%% Save Struct to File
formatOut = 'yyyy.mm.dd.HH.MM';
saveDirectory = [directory1,'Outputs'];
mkdir (saveDirectory);
save([saveDirectory,'\','InitialEstimateCameraParameters.', datestr(now,formatOut), '.mat'],'calibration')
%% Minimization options
minimization_options=optimset('LargeScale','on',...
    'Algorithm','levenberg-marquardt',...
    'TolFun',1e-4,...
    'Display','off',...
    'TolX',1e-8,...
    'MaxFunEvals',20000,...
    'MaxIter',1000,...
    'UseParallel', true);

% Optimization options: fixedK, fixedDistortions, fixedRTcnc1, fixedRTpnc1
options = [1, 1, 0 ,0];
%World and Image Points
genPoints = [calibration.CameraParameters{1}.WorldPoints, ones(size(calibration.CameraParameters{1}.WorldPoints,1),1)];
detPoints = cell2mat(calibration.ImagePointsCommon);
%% Convert initial estimate
serialCalib = serializeCalib(calibration, options);
[calibrationConverted] = unserializeCalib(serialCalib, options, calibration);
errorInitial = CalculateCost(serialCalib, options, calibration, genPoints, detPoints);
tmp = errorInitial; tmp(tmp<0.000001)=NaN; MREi = nanmean(tmp(:));
disp (['Mean Reprojection Error - Before Optimization: ', num2str(MREi)]);
%% Optimization
h = waitbar(0, 'Running global Optimization...','Name', 'Global Optimization');

[optimizedSerialCalib,~,residual,~,~,~,jacobian] = lsqnonlin(@(x) CalculateCost(x, options, calibration, genPoints, detPoints),serialCalib, [],[],minimization_options);

delete(h);

serialCI = nlparci(optimizedSerialCalib,residual,'jacobian',jacobian);
CI(:,1) = unserializeCalib(serialCI(:,1)', options, calibration);
CI(:,2) = unserializeCalib(serialCI(:,2)', options, calibration);

errorOptimized = CalculateCost(optimizedSerialCalib, options, calibration, genPoints, detPoints);
tmp = errorOptimized; tmp(tmp<0.000001)=NaN; MREf = nanmean(tmp(:));
disp (['Mean Reprojection Error - After Optimization: ', num2str(MREf)]);

[calibrationOptimized] = unserializeCalib(optimizedSerialCalib, options, calibration);
%% Save to Struct
calibrationOptimized.OptimizationOptions = options;
calibrationOptimized.Jacobian = jacobian;
calibrationOptimized.Error = errorOptimized;
calibrationOptimized.CI = CI;
%% Save to File
formatOut = 'yyyy.mm.dd.HH.MM';
saveDirectory = [directory1,'Outputs.'];
mkdir (saveDirectory);
save([saveDirectory,'\','optimizedCameraParameters', datestr(now,formatOut), '.mat'],'calibrationOptimized');
disp(['Calibration saved to file: ', saveDirectory,'\','optimizedCameraParameters', datestr(now,formatOut), '.mat'])