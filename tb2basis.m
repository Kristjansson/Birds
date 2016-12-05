function [ex, ey, ez] = tb2basis(yaw, pitch, roll)
    RzYaw = [cos(yaw) -sin(yaw) 0;
        sin(yaw) cos(yaw) 0;
        0 0 1];
    RyPitch = [cos(pitch) 0 -sin(pitch);
        0 1 0;
        sin(pitch) 0 cos(pitch)];
    RxRoll = [1 0 0;
        0 cos(roll) sin(roll);
        0 -sin(roll) cos(roll)];
    temp = RzYaw * RyPitch * RxRoll;
    ex = temp(:, 1);
    ey = temp(:, 2);
    ez = temp(:, 3);
end