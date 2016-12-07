close all; clear all; clc;

X = 1;
Y = 2;
Z = 3;

loadConstants;
setInitialConditions;

DEBUG_LIFT = zeros(3, NumBirds, NumTimeSteps);
DEBUG_DRAG = zeros(3, NumBirds, NumTimeSteps);
DEBUG_THRUST = zeros(3, NumBirds, NumTimeSteps);
DEBUG_GRAVITY = zeros(3, NumBirds, NumTimeSteps);
DEBUG_GAMMA = zeros(NumBirds, NumTimeSteps);
DEBUG_RADIUS = zeros(NumBirds, NumTimeSteps);
DEBUG_SEPARATION_FORCE = zeros(3, NumBirds, NumTimeSteps);
% deltaVecs(:, itr, jtr) is the vector from bird itr to bird jtr.
deltaVecs = zeros(3, NumBirds, NumBirds);
% deltaDists(:, itr, jtr) is the distance between bird itr and bird jtr.
deltaDists = zeros(NumBirds, NumBirds);

epsilon = 1e-12;

%% - Simulation
tic;
for timeStep=1:NumTimeSteps
    for itr=1:NumBirds
        for jtr=1:NumBirds
            deltaVecs(:, itr, jtr) = posOverTime(:, jtr, timeStep) - posOverTime(:, itr, timeStep);
            deltaDists(itr, jtr) = norm(deltaVecs(:, itr, jtr));
        end
    end
    forwardDirections = velOverTime(:, :, timeStep) ./ sqrt(sum(velOverTime(:, :, timeStep).^2));
    % truncate small values to zero to avoid error when computing inverse
    % tangents. 
%     velOverTime(:, :, timeStep) = velOverTime(:, :, timeStep) .* ...
%         double(abs(velOverTime(:, :, timeStep)) > epsilon);
    for bird=1:NumBirds
        % Bird Properties \/==================
        fwdDir = forwardDirections(:, bird);
        [~, wingDir, upDir] = fwdDirAndBeta2basis(fwdDir, bankingOverTime(bird, timeStep));
        speed = norm(velOverTime(:, bird, timeStep));        
        position = posOverTime(:, bird, timeStep);

        interRadius = interactionRadiusOverTime(bird, timeStep);
        neighbors = [1:NumBirds] .* double(deltaDists(bird,:) < interRadius);
        neighbors = neighbors(neighbors~=0);
        neighbors = neighbors(neighbors~=bird);
        angles = acos(squeeze(sum(deltaVecs(:, bird, neighbors).*fwdDir))'./squeeze(deltaDists(bird, neighbors)));
        inView = double(angles < 3*pi/4) + double(angles > 5*pi/4);
        inViewNeighbors = neighbors(inView ~= 0);
        reducedNeighbors = [1:NumBirds] .* double(deltaDists(bird, :) < (2 * interRadius));
        reducedNeighbors = reducedNeighbors(reducedNeighbors ~= 0);
        reducedNeighbors = reducedNeighbors(reducedNeighbors ~= bird);
       
        DEBUG_RADIUS(bird, timeStep) = interRadius;
        % Bird Properites /\==================
        % Interaction Forces \/===============
        g = exp(-(deltaDists(bird,neighbors) - rh).^2 / sigma^2)...
            .*double(deltaDists(bird,neighbors) > rh) ...
            + double(deltaDists(bird,neighbors) <= rh);
        separationForce = -ws/safe_length(neighbors) * sum(g .* squeeze(deltaVecs(:,bird,neighbors))./repmat(deltaDists(bird, neighbors), 3, 1), 2);
        DEBUG_SEPARATION_FORCE(:, bird, timeStep) = separationForce;
        
        centrality = 1/(safe_length(reducedNeighbors)) * norm(sum(squeeze(deltaVecs(:, bird, reducedNeighbors)), 2));
        thresholdDistances = double(deltaDists(bird, inViewNeighbors) > rh);
        cohesion = centrality * ...
            wc/safe_length(inViewNeighbors) * ...
            squeeze(deltaVecs(:, bird, inViewNeighbors))*thresholdDistances'; % This part translates in the summation in equation 5.
        
        diffInDirections = sum(forwardDirections(:, inViewNeighbors) - fwdDir, 2);
        if norm(diffInDirections) > 0
            alignmentForce = wa * diffInDirections / norm(diffInDirections);
        else
            alignmentForce = [0;0;0];
        end
        % Interaction Forces /\===============
        % SteeringForces \/===================
        speedControlForce = mass/Tau * (v0 - speed) * fwdDir;
        altitudeControlForce = -wAlt * (position(Z) - z0) * [0;0;1];
        
        % Attraction To Roost
        normalToRoost = [position(1:2)/norm(position(1:2)); 0];
        roostAttractionForce = -sign(dot(normalToRoost, wingDir)) * WRoost * ...
            (.5 + .5 * dot(fwdDir, normalToRoost)) * wingDir;
        
        % Random Force
        randVec = (2 * [rand; rand; rand] - [1;1;1]);
        randomForce = wSigma * randVec / norm(randVec);
        
        steeringForce = speedControlForce + altitudeControlForce ...
            + roostAttractionForce + randomForce + separationForce + cohesion + alignmentForce;
        % SteeringForces /\===================
        % FlightForces \/=====================
        liftForce = speed^2/v0^2 * L0 * upDir;
        dragForce = -CD_CL * speed^2/v0^2 * mass * gravity * fwdDir;
        thrustForce = T0 * fwdDir;
        gravityForce = [0;0; -mass * gravity];
        
        DEBUG_LIFT(:, bird,timeStep) = liftForce;
        DEBUG_DRAG(:, bird, timeStep) = dragForce;
        DEBUG_THRUST(:, bird, timeStep) = thrustForce;
        DEBUG_GRAVITY(:, bird, timeStep) = gravityForce;
        
        flightForce = liftForce + dragForce + thrustForce + gravityForce;
%         flightForce = [0;0;0];
        % FlightForces /\=====================
        DEBUG_FLAG = position(2) < 0;
        force = steeringForce + flightForce;

        % Banking Angle Equations
        bankingIn = atan(wBin * dot(steeringForce, wingDir) / mass * dt); 
        bankingOut = atan(wBout*sin(bankingOverTime(bird, timeStep))*dt);
        
        % update equations
        velOverTime(:, bird, timeStep + 1) = velOverTime(:, bird, timeStep) + force/mass * dt;
        posOverTime(:, bird, timeStep + 1) = posOverTime(:, bird, timeStep) + velOverTime(:, bird, timeStep + 1) * dt;
        bankingOverTime(bird, timeStep + 1) = bankingOverTime(bird, timeStep) + bankingIn - bankingOut;
        interactionRadiusOverTime(bird, timeStep + (du/dt)) = max(interRadius + ...
            wr * (1 - safe_length(neighbors)/nc) * (Rmax - interRadius) * du, 0);
        if(bird == 1)
%             subplot(3, 1, 1)
%             scatter(timeStep, dot(fwdDir, wingDir)); hold on;
%             subplot(3, 1, 2)
%             scatter(timeStep, dot(fwdDir, upDir)); hold on;
%             subplot(3, 1, 3)
%             scatter(timeStep, dot(upDir, wingDir)); hold on;
        end
    end
    toc;
end
for bird=1:NumBirds
    plot3(squeeze(posOverTime(X,bird,:)), ...
        squeeze(posOverTime(Y,bird,:)),...
        squeeze(posOverTime(Z,bird,:)),'b'); hold on;
    scatter3(squeeze(posOverTime(X,bird,1)), ...
        squeeze(posOverTime(Y,bird,1)),...
        squeeze(posOverTime(Z,bird,1)),'go'); hold on;
    scatter3(squeeze(posOverTime(X,bird,end)), ...
        squeeze(posOverTime(Y,bird,end)),...
        squeeze(posOverTime(Z,bird,end)),'r*'); hold on;
end

fprintf('Survival rate is %f\n', 1 - sum(sum(double(isnan(posOverTime(:,:,end)))))/(3*NumBirds));

figure
plot(sqrt(sum(squeeze(velOverTime(:, 1, :)).^2)), 'r'); hold on;
% plot(sqrt(sum(squeeze(velOverTime(:, 2, :)).^2)), 'g'); hold on;
% plot(sqrt(sum(squeeze(velOverTime(:, 3, :)).^2)), 'b'); hold on;
% figure
% plot(1:NumTimeSteps, DEBUG_FORCES);
% figure
% plot3(-100:100, zeros(201), 10*ones(1,201), 'r')
% plot3(zeros(201), -100:100, 10*ones(1,201), 'g')
figure
plot(1:NumTimeSteps, DEBUG_RADIUS(1, :))