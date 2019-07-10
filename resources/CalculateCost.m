function [cost] = CalculateCost(calib, options, refCalibration, genPoints, detPoints)

[calibrationConverted] = unserializeCalib(calib, options, refCalibration);

cameraParams= calibrationConverted.CameraParameters;
RTcnc1 = calibrationConverted.Extrinsics;
RTpnc1 = calibrationConverted.PatternPositions;

RTpnc1 = calibrationConverted.PatternPositions;

cost = [];
RTcnc1{1} = eye(4,4);

for i = 1: numel(RTcnc1) % for each camera
    for n = 1: numel(RTpnc1) % for each image
 
        points = detPoints(:,2*i-1:2*i,n);
        
        rR =RTcnc1{i}(1:3, 1:3);
        rt = RTcnc1{i}(4, 1:3)';
        Rext =RTpnc1{n}(1:3, 1:3);
        text = RTpnc1{n}(4, 1:3)';
        
        K = cameraParams{i}.IntrinsicMatrix;
        
        if (all(isnan(points(:))) == 0 & all(all(Rext))~=0)
            
            R = Rext*rR;
            t = (rt +rR'*text)';
            
            
            cameraMatrix = [R; t(:)'] * K;
            r_dist = cameraParams{i}.RadialDistortion;
            t_dist = cameraParams{i}.TangentialDistortion;
            
            projectedPointsraw = [genPoints ones(size(genPoints, 1), 1)] * cameraMatrix;
            
            projectedPointsnorm = bsxfun(@rdivide, projectedPointsraw(:, 1:2), projectedPointsraw(:, 3));
            
            distortedPoints = vision.internal.calibration.distortPoints(projectedPointsnorm, K, r_dist, t_dist);
            
        
            difference = (distortedPoints - points);
            error = sqrt(sum((difference).^2, 2));
            cost = [cost; error];
            
         
        end
    end
end
end