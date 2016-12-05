close all; clear all; clc;

X = 1;
Y = 2;
Z = 3;

NumBirds = 1;
NumTimeSteps = 10000;

posOverTime = zeros(3, NumBirds, NumTimeSteps);
velOverTime = zeros(3, NumBirds, NumTimeSteps);

loadConstants;

epsilon = 1e-12;

%% - Initial Conditions
posOverTime(:, 1, 1) = [0;0;100];
% velOverTime(:, 1, 1) = v0*[1;0;1]/norm([1;0;1]);
% velOverTime(:, 2, 1) = (v0 - 0.009)*[1;0;0];
temp = 2*[rand;rand;0] - [1; 1; 0]
velOverTime(:, 1, 1) = 2*v0*temp / norm(temp);
% velOverTime(:, 1, 1) = [1;1;1];
% should result in ex = [1 1 1], ey = [1 -1 0], ez = [-1 -1 2]
bankingOverTime(1, 1) = 0;

DEBUG_FORCES = zeros(NumBirds, NumTimeSteps);
DEBUG_GAMMA = zeros(NumBirds, NumTimeSteps);

%% - Simulation
for timeStep=1:NumTimeSteps
    % truncate small values to zero to avoid error when computing inverse
    % tangents. 
%     velOverTime(:, :, timeStep) = velOverTime(:, :, timeStep) .* ...
%         double(abs(velOverTime(:, :, timeStep)) > epsilon);
    for bird=1:NumBirds
        alpha = atan(velOverTime(Y, bird, timeStep) / velOverTime(X, bird, timeStep));
        beta = bankingOverTime(bird, timeStep);
        gamma = atan(velOverTime(Z, bird, timeStep) / ...
            sqrt(velOverTime(X, bird, timeStep)^2 + velOverTime(Y, bird, timeStep)^2));
        DEBUG_GAMMA(bird, timeStep) = gamma;
        fwdDir = velOverTime(:, bird, timeStep) / norm(velOverTime(:, bird, timeStep));
        [~, wingDir, upDir] = tb2basis(alpha, gamma, beta);
        speed = norm(velOverTime(:, bird, timeStep));        
     
        % SteeringForces \/===================        
%         speedControlForce = mass/Tau * (v0 - speed) * fwdDir;
%         steeringForce = speedControlForce;
        steeringForce = [0;0;0];
        % SteeringForces /\===================
        % FlightForces \/=====================
        liftForce = speed^2/v0^2 * L0 * upDir;
        dragForce = -CD_CL * speed^2/v0^2 * mass * g * fwdDir;
        thrustForce = T0 * fwdDir;
        gravityForce = [0;0; -mass * g];
        DEBUG_FORCES(bird,timeStep) = norm(liftForce + gravityForce);
        flightForce = liftForce + dragForce + thrustForce + gravityForce;
%         flightForce = [0;0;0];
        % FlightForces /\=====================
        force = steeringForce + flightForce;

        % Banking Angle Equations
        bankingIn = 0; 
        bankingOut = 0;
        
        % update equations
        velOverTime(:, bird, timeStep + 1) = velOverTime(:, bird, timeStep) + force/mass * dt;
        posOverTime(:, bird, timeStep + 1) = posOverTime(:, bird, timeStep) + velOverTime(:, bird, timeStep + 1) * dt;
        bankingOverTime(bird, timeStep + 1) = bankingOverTime(bird, timeStep) + bankingIn - bankingOut;

        if(bird == 1)
%             subplot(3, 1, 1)
%             scatter(timeStep, dot(fwdDir, wingDir)); hold on;
%             subplot(3, 1, 2)
%             scatter(timeStep, dot(fwdDir, upDir)); hold on;
%             subplot(3, 1, 3)
%             scatter(timeStep, dot(upDir, wingDir)); hold on;
        end
    end
end
for bird=1:NumBirds
    plot3(squeeze(posOverTime(X,bird,:)), ...
        squeeze(posOverTime(Y,bird,:)),...
        squeeze(posOverTime(Z,bird,:)),'b'); hold on;
%     scatter3(squeeze(posOverTime(X,bird,:)), ...
%         squeeze(posOverTime(Y,bird,:)),...
%         squeeze(posOverTime(Z,bird,:)),'bo'); hold on;
    scatter3(squeeze(posOverTime(X,bird,end)), ...
        squeeze(posOverTime(Y,bird,end)),...
        squeeze(posOverTime(Z,bird,end)),'b*'); hold on;
end

figure
plot(sqrt(sum(squeeze(velOverTime(:, 1, :)).^2)), 'r'); hold on;
% plot(sqrt(sum(squeeze(velOverTime(:, 2, :)).^2)), 'g'); hold on;
% plot(sqrt(sum(squeeze(velOverTime(:, 3, :)).^2)), 'b'); hold on;
figure
plot(1:NumTimeSteps, DEBUG_GAMMA);