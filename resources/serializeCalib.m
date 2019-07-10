function x = serializeCalib(calibration, options)
        cameraParams= calibration.CameraParameters;
        RTcnc1 = calibration.Extrinsics;
        RTpnc1 = calibration.PatternPositions;
        numCams = numel(cameraParams);        
        
        % options (fixedK, fixedDistortions, fixedRTpnc1, fixedRTcnc1)
        x = [];
        for i=1:numCams
            % x = [fx; fy; cx; cy; skew; radial; tangential; rvecs; tvecs];
            if (~options(1))
                x = [x;[cameraParams{i}.IntrinsicMatrix(1,1); cameraParams{i}.IntrinsicMatrix(2,2); cameraParams{i}.IntrinsicMatrix(3,1); cameraParams{i}.IntrinsicMatrix(3,2)]];
                
                if cameraParams{i}.EstimateSkew
                    x = [x; cameraParams{i}.IntrinsicMatrix(1,2)];
                end
            end
            if (~options(2))
                x = [x; cameraParams{i}.RadialDistortion(1:cameraParams{i}.NumRadialDistortionCoefficients)'];
                
                if cameraParams{i}.EstimateTangentialDistortion
                    x = [x; cameraParams{i}.TangentialDistortion'];
                end
            end
        end
        % Extrinsics
        % x = [fx; fy; cx; cy; skew; radial; tangential; rvecs; tvecs; A; B; C; Tx; Ty; Tz];
        for i=2:numCams
            if (~options(3))
                rvec = rotationMatrixToVector(RTcnc1{i}(1:3, 1:3));
                x = [x; rvec'; RTcnc1{i}(4, 1:3)'];
            end
            numel(x);
        end
        
        if (~options(4))
            for i=1:numel(RTpnc1)
                rvec = rotationMatrixToVector(RTpnc1{i}(1:3, 1:3));
                x = [x; rvec(:); RTpnc1{i}(4, 1:3)'];
            end
            numel(x);
        end
end