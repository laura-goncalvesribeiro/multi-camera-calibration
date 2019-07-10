        function [newCalibration] = unserializeCalib(x, options, refCalibration)
        numCams = numel(refCalibration.CameraParameters);
        
        % Start with original values. Overwrite if changed.
        newCalibration=refCalibration;
        RTcnc1 = refCalibration.Extrinsics;
        RTpnc1 = refCalibration.PatternPositions;
        
        %options (fixedK, fixedDistortions, fixedRTpnc1, fixedRTcnc1)
        camLength = 0;
        for i =0:numCams-1
            if (~options(1))
                K = [[x(camLength+1),0,0];[0,x(camLength+2),0];[x(camLength+3),x(camLength+4),1]];
                camLength = camLength+4;
            else 
                K = refCalibration.CameraParameters{i+1}.IntrinsicMatrix;
            end
            if (~options(2))
                radialCoeffs = [x(camLength+1), x(camLength+2), x(camLength+3)];
                camLength = camLength+3;
            else
                radialCoeffs = refCalibration.CameraParameters{i+1}.RadialDistortion;
            end
            newCalibration.CameraParameters{i+1} = cameraParameters('IntrinsicMatrix', K, ...
                'RadialDistortion', radialCoeffs,  'NumRadialDistortionCoefficients', 3);
        end
        if (~options(3))
            for i =1:numCams-1
                RTcnc1{i+1}=RTtoTransform(rotationVectorToMatrix(x(camLength+1:camLength+3)),x(camLength+4: camLength+6));
                camLength = camLength +6;
            end
        end
        
        if (~options(4))
            for i=0:numel(RTpnc1)-1
                RTpnc1{i+1} = RTtoTransform(rotationVectorToMatrix (x(camLength+1:camLength+3)), x(camLength+4: camLength+6));
                camLength = camLength +6;
            end
        end
        

        numel(x);
        
        newCalibration.Extrinsics = RTcnc1;
        newCalibration.PatternPositions = RTpnc1;
        
        end