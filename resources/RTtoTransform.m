% Before applying transform all R's coming from outside Matlab
function transform = RTtoTransform(R, T)    
    transform(1:3, 1:3) = R;
    transform(4,1:3) = T;
    transform(1:4,4) = [0,0,0,1];
end