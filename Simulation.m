close all; clear all; clc;

X = 1;
Y = 2;
Z = 3;

NumBirds = 3;
NumTimeSteps = 100;

posOverTime = zeros(3, NumBirds, NumTimeSteps);
velOverTime = zeros(3, NumBirds, NumTimeSteps);
bankingOverTime = zeros(NumBirds, NumTimeSteps);

loadConstants;

%% - Initial Conditions
posOverTime(:, 1, 1) = [0;0;0];
velOverTime(:, 1, 1) = [1;1;1];
velOverTime(:, 2, 1) = [1;0;0];
velOverTime(:, 3, 1) = [0;1;0];
% velOverTime(:, 1, 1) = [1;1;1];
% should result in ex = [1 1 1], ey = [1 -1 0], ez = [-1 -1 2]
bankingOverTime(1, 1) = 0;

%% - Simulation
for timeStep=1:NumTimeSteps
    for bird=1:NumBirds
        alpha = atan(velOverTime(Y, bird, timeStep) / velOverTime(X, bird, timeStep));
        beta = bankingOverTime(bird, NumTimeSteps);
        gamma = atan(velOverTime(Z, bird, timeStep) / ...
            sqrt(velOverTime(X, bird, timeStep)^2 + velOverTime(Y, bird, timeStep)^2));
        [fwdDir, wingDir, upDir] = tb2basis(alpha, gamma, beta);
        speed = norm(velOverTime(:, bird, timeStep));        
     
        % SteeringForces \/===================        
        speedControlForce = mass/T * (v0 - speed) * fwdDir;
        steeringForce = speedControlForce;
        % SteeringForces /\===================
        % FlightForces \/=====================
        liftForce = speed^2/v0^2 * mass * g * upDir;
        dragForce = -CL_CD * speed^2/v0^2 * mass * g * fwdDir;
        thrustForce = T0 * fwdDir;
        gravityForce = [0;0;-mass * g];
        flightForce = liftForce + dragForce + thrustForce + gravityForce;
        % FlightForces /\=====================
        force = steeringForce + flightForce;
        mass = 1;
        
        % Banking Angle Equations
        bankingIn = 0; 
        bankingOut = 0;
        
        % update equations
        velOverTime(:, bird, timeStep + 1) = velOverTime(:, bird, timeStep) + force/mass * dt;
        posOverTime(:, bird, timeStep + 1) = posOverTime(:, bird, timeStep) + velOverTime(:, bird, timeStep + 1) * dt;
        bankingOverTime(bird, timeStep + 1) = bankingOverTime(bird, timeStep) + bankingIn - bankingOut;
        
    end
end