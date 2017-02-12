function [ey, ez] = fwdDirAndBeta2basis(fwdDir, beta)
    % Assume the fwdDir is normalized!
    normxy = norm(fwdDir(1:2));
    cosyaw = fwdDir(1) / normxy;
    sinyaw = fwdDir(2) / normxy;
    if isnan(cosyaw) || isnan(sinyaw)
        % assumes birds are never vertical 'gimbal lock'
        cosyaw = 0;
        sinyaw = sign(fwdDir(2)); 
    end
    
    cospitch = norm(fwdDir(1:2));
    sinpitch = fwdDir(3);
    if isnan(cospitch) || isnan(sinpitch)
        cospitch = sign(fwdDir(1));
        sinpitch = 0;
    end
    cosroll = cos(beta);
    sinroll = sin(beta);

    ey = [sinroll*sinpitch*cosyaw - sinyaw*cosroll;
        sinroll*sinpitch*sinyaw + cosyaw*cosroll;
        -sinroll*cospitch];
    ez = [-cosroll*sinpitch*cosyaw - sinroll*sinyaw;
        -cosroll*sinpitch*sinyaw + cosyaw*sinroll;
        cosroll*cospitch];
end