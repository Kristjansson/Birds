close all; clear all; clc;

loadConstants;
setInitialConditions;

% deltaVecs(:, itr, jtr) is the vector from bird itr to bird jtr.
deltaVecs = zeros(3, NumBirds, NumBirds);
% deltaDists(:, itr, jtr) is the distance between bird itr and bird jtr.
deltaDists = zeros(NumBirds, NumBirds);
speeds=zeros(NumBirds, 1);

%% - Simulation
tic;
for timeStep=1:NumTimeSteps
    for itr=1:NumBirds
        for jtr=1:NumBirds
            deltaVecs(:, itr, jtr) = posOverTime(:, jtr, timeStep) - ...
                posOverTime(:, itr, timeStep);
            deltaDists(itr, jtr) = norm(deltaVecs(:, itr, jtr));
        end
    end
    speeds = sqrt(sum(velOverTime(:, :, timeStep).^2));
    forwardDirections = velOverTime(:, :, timeStep) ./ speeds;
    for bird=1:NumBirds
        % Bird Properties \/==================
        fwdDir = forwardDirections(:, bird);
        [wingDir, upDir] = fwdDirAndBeta2basis(fwdDir, ...
            bankingOverTime(bird, timeStep));
        speed = speeds(bird);        
        position = posOverTime(:, bird, timeStep);

        interRadius = interactionRadiusOverTime(bird, timeStep);
        neighbors = [1:NumBirds] .* double(...
            deltaDists(bird,:) < interRadius);
        neighbors = neighbors(neighbors~=0);
        neighbors = neighbors(neighbors~=bird);
        angles = acos(squeeze(sum(...
            deltaVecs(:, bird, neighbors).*fwdDir))'./squeeze(...
            deltaDists(bird, neighbors)));
        inView = double(angles < 3*pi/4) + double(angles > 5*pi/4);
        inViewNeighbors = neighbors(inView ~= 0);
        reducedNeighbors = [1:NumBirds] .* double(deltaDists(bird, :)...
             < (2 * interRadius));
        reducedNeighbors = reducedNeighbors(reducedNeighbors ~= 0);
        reducedNeighbors = reducedNeighbors(reducedNeighbors ~= bird);
        
        % Bird Properites /\==================
        % Interaction Forces \/===============
        g = exp(-(deltaDists(bird,neighbors) - rh).^2 / sigma^2)...
            .*double(deltaDists(bird,neighbors) > rh) ...
            + double(deltaDists(bird,neighbors) <= rh);
        separationForce = -ws/safe_length(neighbors) * sum(g .* ...
            squeeze(deltaVecs(:,bird,neighbors))./repmat(...
            deltaDists(bird, neighbors), 3, 1), 2);
        
        centrality = 1/(safe_length(reducedNeighbors)) * norm(sum(...
            squeeze(deltaVecs(:, bird, reducedNeighbors)), 2));
        thresholdDistances = double(deltaDists(bird, inViewNeighbors)...
            > rh);
        cohesion = centrality * ...
            wc/safe_length(inViewNeighbors) * squeeze(...
            deltaVecs(:, bird, inViewNeighbors))*thresholdDistances'; 
        
        diffInDirections = sum(forwardDirections(:, inViewNeighbors)...
            - fwdDir, 2);
        if norm(diffInDirections) > 0
            alignmentForce = wa * diffInDirections / ...
                norm(diffInDirections);
        else
            alignmentForce = [0;0;0];
        end
        % Interaction Forces /\===============
        % SteeringForces \/===================
        speedControlForce = mass/Tau * (v0 - speed) * fwdDir;
        altitudeControlForce = -wAlt * (position(Z) - z0) * [0;0;1];

        % Attraction To Roost
        normalToRoost = [position(1:2);0];
        normalToRoost = normalToRoost/norm(normalToRoost);
        roostAttractionForce = -sign(dot(normalToRoost, wingDir)) * ...
            wRoost * (.5 + .5 * dot(fwdDir, normalToRoost)) * wingDir;
        
        % Random Force
        randVec = (2 * [rand; rand; rand] - [1;1;1]);
        randomForce = wSigma * randVec / norm(randVec);
        
        steeringForce = speedControlForce + altitudeControlForce + ...
            roostAttractionForce + randomForce + separationForce + ...
            cohesion + alignmentForce;
        
        % SteeringForces /\===================
        % FlightForces \/=====================
        liftForce = liftConstant * speed^2* upDir;
        dragForce = dragConstant * speed^2 * fwdDir;
        thrustForce = T0 * fwdDir;
        % gravityForce never changes and so is precomputed in the loading
        % constants phase.         
        flightForce = liftForce + dragForce + thrustForce + gravityForce;
        % FlightForces /\=====================
        force = steeringForce + flightForce;

        % Banking Angle Equations
        bankingIn = atan(wBin * dot(steeringForce, wingDir) / mass * dt); 
        bankingOut = atan(wBout*sin(bankingOverTime(bird, timeStep))*dt); 
        % update equations
        velOverTime(:, bird, timeStep + 1) = ...
            velOverTime(:, bird, timeStep) + force/mass * dt;
        posOverTime(:, bird, timeStep + 1) = ...
            posOverTime(:, bird, timeStep) + ...
            velOverTime(:, bird, timeStep + 1) * dt;
        interactionRadiusOverTime(bird, timeStep + (du/dt)) = max(...
            interRadius + wr * (1 - safe_length(neighbors)/nc) * ...
            (Rmax - interRadius) * du, 0);
        bankingOverTime(bird, timeStep + 1) = median([...
            bankingOverTime(bird, timeStep) + bankingIn - bankingOut;
            -pi/2;
            pi/2]);
    end
    toc;
    if mod(timeStep, 100) == 0
        fprintf('Completed Timestep %d', timeStep);
    end 
end