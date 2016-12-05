function [ex, ey, ez] = fwdDirAndBeta2basis(fwdDir, beta)
    normxy = norm(fwdDir(1:2));
    cosyaw = fwdDir(1) / normxy;
    sinyaw = fwdDir(2) / normxy;
    if isnan(cosyaw) || isnan(sinyaw)
        % assumes birds are never vertical 'gimbal lock'
        cosyaw = 0;
        sinyaw = sign(fwdDir(2));    
    end
    normxz = sqrt(fwdDir(1)^2 + fwdDir(3)^2);
    cospitch = fwdDir(1) / normxz;
    sinpitch = fwdDir(3) / normxz;
    if isnan(cospitch) || isnan(sinpitch)
        cospitch = 1;
        sinpitch = 0;
    end
    cosroll = cos(beta);
    sinroll = sin(beta);

    RzYaw = [cosyaw -sinyaw 0;
        sinyaw cosyaw 0;
        0 0 1];
    RyPitch = [cospitch 0 -sinpitch;
        0 1 0;
        sinpitch 0 cospitch];
    RxRoll = [1 0 0;
        0 cosroll sinroll;
        0 -sinroll cosroll];

    temp = RzYaw * RyPitch * RxRoll;
    ex = temp(:, 1);
    ey = temp(:, 2);
    ez = temp(:, 3);
end