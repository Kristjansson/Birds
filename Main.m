close all; clear all; clc;

ForceConversion = 1e-06; % 1 Newton => 1e-06; % kg m/(ms)^2
% Parameters From Paper
% g = 9.8; %m/s^2     gravity
global CONSTANTS;
global BIRD;
CONSTANTS = struct(...
    'g', 9.8e-6, ... %m/ms^2 gravity
    'dt', 5, ... %ms         integration time step
    'du', 50, ... %ms        Reaction time !!check s if you change!!
    'v0', 0.01, ... %m/ms    Cruise Speed
    'M', 0.08, ... %kg       Mass
    'CL_CD', 3.3, ... %      Lift-drag coefficient
    'L0', 0.78, ... % N      Default Lift
    'D0', 0.24, ... %N       Default Drag
    'T0', 0.24, ... %N       Default Thrust
    'wBin', 10, ... %        Banking Control
    'wBout', 1, ... %        Banking Control
    'T', 62.5, ... %s        Speed Control
    'Rmax', 100, ... %m      Maximum Percetion Radius
    'nc', 6.5, ... %         Topological Range
    's', 0.1, ... %          Interpolation Factor !!50 is du!!
    'rh', 0.2, ... %m        Radius of Max Separation
    'rsep', 4, ... %m        Seperation Radius
    'sigma', 1.37, ... %m    Parameter of the Gaussian
    'ws', 1, ... %N          Weighting factor separation force
    'blindAngle', pi/2, ... %Rear "blind angle" cohesion and alignment
    'wa', 0.5, ... %N        Weighting factor alignment force
    'wc', 1, ... %N          Weighting factor cohesion force
    'Cc', 0.35, ... %        Critical Centrality 
    'wSigma', 0.01,    ... %N   Weight factor random force
    'RRoost', 150, ... %m    Radius of Roost
    'WRoostH', 0.01, ... %N/m Weighting factor horizontal attraction to roost
    'WRoostV', 0.2 ... %N    Weighting factor vertical attraction to roost
);
%Convert Force Constants;
CONSTANTS.L0 = CONSTANTS.L0 * ForceConversion;
CONSTANTS.D0 = CONSTANTS.D0 * ForceConversion;
CONSTANTS.T0 = CONSTANTS.T0 * ForceConversion;
CONSTANTS.ws = CONSTANTS.ws * ForceConversion;
CONSTANTS.wa = CONSTANTS.wa * ForceConversion;
CONSTANTS.wc = CONSTANTS.wc * ForceConversion;
CONSTANTS.wSigma = CONSTANTS.wSigma * ForceConversion;
CONSTANTS.WRoostH = CONSTANTS.WRoostH * ForceConversion;
CONSTANTS.WRoostV = CONSTANTS.WRoostV * ForceConversion;

% Simulation Parameters
NumBirds = 20;
NumTimeSteps = 2000;
% Bird Storage
BIRD = struct('px', 1, 'py', 2, 'pz', 3, ...
    'v', 4, ...
    'exx', 7, 'exy', 8, 'exz', 9, ...
    'eyx', 10, 'eyy', 11, 'eyz', 12, ...
    'ezx', 13, 'ezy', 14, 'ezz', 15, ...
    'ba', 16, ... %banking angle
    'r', 17); % adaptive interaction range 

TRIALS = 1;

colors = ['r', 'g', 'b', 'c', 'm', 'y', 'k', 'w',];
for trial=1:TRIALS
    tic;
    birdStorage = zeros(length(fieldnames(BIRD)), NumBirds, NumTimeSteps);

%     %% Set Initial Conditions
    initial_cylinder_height = 100;
    initial_cylinder_radius = CONSTANTS.RRoost;
    theta = 2*pi*rand(1, NumBirds);
    height = initial_cylinder_height * rand(1, NumBirds) + 10;
    birdStorage(BIRD.px:BIRD.pz, :, 1) = ...
        [initial_cylinder_radius * cos(theta);...
        initial_cylinder_radius * sin(theta);...
        height];

%     birdStorage(BIRD.vx:BIRD.vz, :, 1) = initial_velocity;
    birdStorage(BIRD.v, :, 1) = 2*CONSTANTS.v0*rand(1, NumBirds);
%     birdStorage(BIRD.v, :, 1) = [0.0091, 0.0079, 0.0101, 0.0002, 0.0080, 0.0098];
    initialFwdDirections = [rand(2, NumBirds); zeros(1, NumBirds)];
    birdStorage(BIRD.exx:BIRD.exz, :, 1) = initialFwdDirections ./ sqrt(sum(initialFwdDirections.^2));
    birdStorage(BIRD.ezx:BIRD.ezz, :, 1) = [zeros(2, NumBirds); ones(1, NumBirds)];
    
    for itr = 1:NumBirds
        birdStorage(BIRD.eyx:BIRD.eyz, itr, 1) = cross(...
            birdStorage(BIRD.exx:BIRD.exz, itr, 1), ...
            birdStorage(BIRD.ezx:BIRD.ezz, itr, 1));
    end

    birdStorage(BIRD.r, :, 1:CONSTANTS.du / CONSTANTS.dt) = CONSTANTS.RRoost^2;

    for t=1:NumTimeSteps-1
%         tic
        for itr=1:NumBirds
            % Compute Neighborhoods and Interaction Range
            %% (3) Ni = findNeighborsInRadius(birdStorage, itr, t, birdStorage(BIRD.r, itr, t));
            position = birdStorage(BIRD.px:BIRD.pz, itr, t);
            birds = [1:itr-1, itr+1:size(birdStorage, 2)];
            vectorDistances = position - birdStorage(BIRD.px:BIRD.pz, birds, t);
            distances = sqrt(sum(vectorDistances.^2));
            [sortedDistances, sortedIdxs] = sort(distances);
            % Interfaces with the deltaPositions and distances arrays. wrap
            % it with birds to map to birdStorage
            Ni = sortedIdxs(sortedDistances < birdStorage(BIRD.r, itr, t));
%             Ni = birds(sortedIdxs(distances < radius));
            
            if isempty(Ni)
                Ni = sortedIdxs(1);
            end

            %% (6) reduced neighborhood
            direction = birdStorage(BIRD.exx:BIRD.exz, itr, t);
            angles = acos((sum(vectorDistances(:, Ni).*direction)./distances(Ni)) / norm(direction));
            inView = double(angles < pi - CONSTANTS.blindAngle/2) + double(angles > pi + CONSTANTS.blindAngle/2);
            NiStar = Ni(inView ~= 0);
            if isempty(NiStar)
                NiStar = sortedIdxs(1);
            end
            %% (2) adaptive Interaction Range
            Ri = birdStorage(BIRD.r, itr, t);
            RiP1 = (1 - CONSTANTS.s)*Ri + CONSTANTS.s * ...
                (CONSTANTS.Rmax - (CONSTANTS.Rmax * length(Ni) / CONSTANTS.nc));
            birdStorage(BIRD.r, itr, t + CONSTANTS.du / CONSTANTS.dt) = RiP1;


            %% STEERING FORCE EQUATIONS ==============================================================
            %% (4) separationForce(birdStorage, itr, t, Ni)
            g = exp(-(distances - CONSTANTS.rh).^2 / CONSTANTS.sigma^2)...
                .*double(distances > CONSTANTS.rh) ...
                + double(distances <= CONSTANTS.rh);
            separationForce = -CONSTANTS.ws/length(Ni) * sum(g .* vectorDistances, 2);

            %% (8) alignment(birdStorage, itr, t, NiStar)
            diffInDirections = sum(birdStorage(BIRD.exx:BIRD.exz, birds(NiStar), t) - direction, 2);
            alignment = CONSTANTS.wa * diffInDirections / norm(diffInDirections);

            %% 
            Ng = sortedIdxs(sortedDistances < 2 * birdStorage(BIRD.r, itr, t));
            if isempty(Ng)
                Ng = sortedIdxs(1);
            end

            %% (7) centrality(birdStorage, itr, t)
            centrality = 1/10000 * 1/(length(Ng)) * norm(sum(vectorDistances(:, Ng), 2));

            %% (5) cohesion(birdStorage, itr, t, NiStar)
            thresholdDistances = distances(NiStar) > CONSTANTS.rh;
            cohesion = centrality * ...
                CONSTANTS.wc/length(NiStar) * ...
                vectorDistances(:, NiStar)*thresholdDistances'; % This part translates in the summation in equation 5.

            %% (9) social force
            socialForce = separationForce + ... % Is inf or nan
                alignment + ...
                cohesion;

            %% (10, 11, 12) roostAttraction(birdStorage, itr, t)
            % Still need to incorporate mysterious vector n and plus minus logic.
            horizontalRoostAttraction = CONSTANTS.WRoostH * ...
                (0.5 + 0.5*dot(direction,[1;1;1])) * ...
                birdStorage(BIRD.eyx:BIRD.eyz, itr, t);
            verticalRootAttraction = -CONSTANTS.WRoostV * ...
                birdStorage(BIRD.pz, itr, t) * [0,0,1]';

            roostAttraction = horizontalRoostAttraction + verticalRootAttraction;

            %% (1) speedControl(birdStorage, itr, t)
            speed = birdStorage(BIRD.v, itr, t);
            speedControl = (CONSTANTS.M / CONSTANTS.T) * (CONSTANTS.v0 - speed) * direction;
            %% (13) randomForce()
            randomForce = CONSTANTS.wSigma * [rand;rand;rand];

            %% (14) steeringForce
            steeringForce = socialForce + ...
                roostAttraction + ...
                speedControl + ...
                randomForce;

            steeringForce = speedControl;
%             test_angle = dot(birdStorage(BIRD.vx:BIRD.vz, itr, t), direction) / (speed * norm(direction));
%             fprintf('bird %d has speedControl: %f\n', itr, norm(speedControl));
%             fprintf('bird %d has angle: %f\n', itr, test_angle);
            %% FLIGHT FORCE EQUATIONS ==============================================================
            % (15a, b, c)
            simplifiedLift = speed^2/CONSTANTS.v0^2 * CONSTANTS.M * CONSTANTS.g * ...
                birdStorage(BIRD.ezx:BIRD.ezz, itr, t);
            simplifiedDrag = CONSTANTS.CL_CD*speed^2/CONSTANTS.v0^2 * CONSTANTS.M * ...
                CONSTANTS.g * birdStorage(BIRD.ezx:BIRD.ezz, itr, t);
            thrust = CONSTANTS.T0 * birdStorage(BIRD.exx:BIRD.exz, itr, t);

            % (16)
            flightForce = simplifiedLift + ...
                simplifiedDrag + ...
                thrust + ...
                (CONSTANTS.M * [0;0;-CONSTANTS.g]);

%             flightForce = thrust;
            flightForce = [0;0;0];
            %% (21, 22) update velocity and position
            forceSum = steeringForce + flightForce;
            velocityVectorUpdate = (birdStorage(BIRD.v, itr, t)*direction) + ... %curr velocity
                (CONSTANTS.dt/CONSTANTS.M * forceSum);
            birdStorage(BIRD.v, itr, t+1) = norm(velocityVectorUpdate);
            birdStorage(BIRD.exx:BIRD.exz, itr, t+1) = velocityVectorUpdate/norm(velocityVectorUpdate);
            
            % Update position
            birdStorage(BIRD.px:BIRD.pz, itr, t+1) = birdStorage(BIRD.px:BIRD.pz, itr, t) + ...
                velocityVectorUpdate * CONSTANTS.dt;

            %% (17) lateralAcceleration
            lateralAcceleration = dot(steeringForce, birdStorage(BIRD.eyx:BIRD.eyz, itr, t)) / ...
                CONSTANTS.M * birdStorage(BIRD.eyx:BIRD.eyz, itr, t);

            %% (18, 19, 20) banking angle
            tanBin = CONSTANTS.wBin * norm(lateralAcceleration) * CONSTANTS.dt;
            tanBout = CONSTANTS.wBout * sin(birdStorage(BIRD.ba, itr, t)) * CONSTANTS.dt;
            bap1 = birdStorage(BIRD.ba, itr, t) + atan(tanBin) - atan(tanBout);

            % Update Banking angle and orientation
            birdStorage(BIRD.ba, itr, t+1) = bap1;
            temp_ey = ...
                (cos(bap1) * birdStorage(BIRD.eyx:BIRD.eyz, itr, t) + ...
                sin(bap1) * birdStorage(BIRD.ezx:BIRD.ezz, itr, t));
            birdStorage(BIRD.eyx:BIRD.eyz, itr, t+1) = temp_ey/norm(temp_ey);
            temp_ez = ...
                -1 * (cos(pi - bap1) * birdStorage(BIRD.ezx:BIRD.ezz, itr, t) + ...
                sin(pi - bap1) * birdStorage(BIRD.ezx:BIRD.ezz, itr, t));
            birdStorage(BIRD.ezx:BIRD.ezz, itr, t+1) = temp_ez/norm(temp_ez);
%             if mod(itr, 10) == 0
%                 fprintf('Completed bird %d\n', itr)
%             end
            if(itr == 1)
                subplot(3, 1, 1)
                scatter(t, dot(birdStorage(BIRD.exx:BIRD.exz, itr, t),...
                    birdStorage(BIRD.eyx:BIRD.eyz, itr, t))); hold on;
                subplot(3, 1, 2)
                scatter(t, dot(birdStorage(BIRD.exx:BIRD.exz, itr, t),...
                    birdStorage(BIRD.ezx:BIRD.ezz, itr, t))); hold on;
                subplot(3, 1, 3)
                scatter(t, dot(birdStorage(BIRD.eyx:BIRD.eyz, itr, t),...
                    birdStorage(BIRD.ezx:BIRD.ezz, itr, t))); hold on;
            end
                
        end 
        if mod(t, 50) == 0
            fprintf('Completed timestep %d\n', t)
        end
%         fprintf('====== TIMESTEP======= %d\n', t)
%         toc;
    end
    toc;
%     subplot(2,2,trial)
    figure(2)
    for itr=1:6
        % plot velocity over time.
%         subplot(1,6, itr)
%         plot(1:NumTimeSteps, squeeze(birdStorage(BIRD.v, itr, 1:NumTimeSteps))); hold on;
        plot3(squeeze(birdStorage(BIRD.px, itr, 1:NumTimeSteps)), ...
            squeeze(birdStorage(BIRD.py, itr, 1:NumTimeSteps)), ...
            squeeze(birdStorage(BIRD.pz, itr, 1:NumTimeSteps)), 'b'); hold on;
        scatter3(squeeze(birdStorage(BIRD.px, itr, NumTimeSteps)), ...
            squeeze(birdStorage(BIRD.py, itr, NumTimeSteps)), ...
            squeeze(birdStorage(BIRD.pz, itr, NumTimeSteps)), 'b*'); hold on;
%         for step=1:NumTimeSteps - 1            
%             scatter3(squeeze(birdStorage(BIRD.px, itr, step)), ...
%                 squeeze(birdStorage(BIRD.py, itr, step)), ...
%                 squeeze(birdStorage(BIRD.pz, itr, step)), colors(mod(step, length(colors)) + 1)); hold on;
%         end
    end
end