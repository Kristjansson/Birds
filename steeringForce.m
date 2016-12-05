%% separationForce(birdStorage, itr, t, Ni)
bpi = birdStorage(BIRD.px:BIRD.pz, itr, t);
vectorDistances = bpi - birdStorage(BIRD.px:BIRD.pz, Ni, t);
distances = sqrt(sum(vectorDistances.^2));
g = exp(-(distances - CONSTANTS.rh).^2 / CONSTANTS.sigma^2)...
    .*double(distances > CONSTANTS.rh) ...
    + double(distances <= CONSTANTS.rh);
separationForce = -CONSTANTS.ws/length(Ni) * sum(g .* vectorDistances, 2);

%% alignment(birdStorage, itr, t, NiStar)
fwdDirection = birdStorage(BIRD.exx:BIRD.exz, itr, t);
diffInDirections = sum(birdStorage(BIRD.exx:BIRD.exz, NiStar, t) - fwdDirection, 2);
alignment = CONSTANTS.wa * diffInDirections / norm(diffInDirections);

%% findNeighborsInRadius(birdStorage, itr, t, birdStorage(BIRD.r, itr, t))
radius = 2 * birdStorage(BIRD.r, itr, t);
% Set Ni to be the subset of bird indexes who's corresponding distance
% is less then Ri.
Ng = idxs(distances < radius);
% If Ni is empty, then assign the closest bird index to it.
if isempty(Ng)
    Ng = idxs(1);
end

%% centrality(birdStorage, itr, t)
Ng = findNeighborsInRadius(birdStorage, itr, t, birdStorage(BIRD.r, itr, t));
bpi = birdStorage(BIRD.px:BIRD.pz, itr, t);
centrality = 1/(length(Ng)) * norm(sum(bpi - birdStorage(BIRD.px:BIRD.pz, Ng, t), 2));

%% cohesion(birdStorage, itr, t, NiStar)
% get the current bird position
bpi = birdStorage(BIRD.px:BIRD.pz, itr, t);
% get the distances from each bird to other birds in NiStar
distVecs = (bpi - birdStorage(BIRD.px:BIRD.pz, NiStar, t));
% compute if distances norms are above the CONSTANT.rh, the hard sphere
thresholdDistances = sqrt(sum(distVecs.^2)) > CONSTANTS.rh;
fci = centrality * ...
    CONSTANTS.wc/length(NiStar) * ...
    distVecs*thresholdDistances'; % This part translates in the summation in equation 5.

%% social force
socialForce = separationForce + ... % Is inf or nan
    alignment + ...
    cohesion;

%% roostAttraction(birdStorage, itr, t)
% Still need to incorporate mysterious vector n and plus minus logic.
horizontalRoostAttraction = CONSTANTS.WRoostH * ...
    (0.5 + 0.5*dot(birdStorage(BIRD.exx:BIRD.exz, itr, t),[1;1;1])) * ...
    birdStorage(BIRD.eyx:BIRD.eyz, itr, t);
verticalRootAttraction = -CONSTANTS.WRoostV * ...
    birdStorage(BIRD.pz, itr, t) * [0,0,1]';

roostAttraction = horizontalRoostAttraction + verticalRootAttraction

%% speedControl(birdStorage, itr, t)
fwdDirection = birdStorage(BIRD.exx:BIRD.exz, itr, t);
speed = norm(birdStorage(BIRD.vx:BIRD.vz, itr, t));
speedControl = (CONSTANTS.M / CONSTANTS.T) * (CONSTANTS.v0 - speed) * fwdDirection;

%% randomForce()
rv = CONSTANTS.wSigma * [rand;rand;rand];

%% steeringForce
steeringForce = socialForce + ...
    roostAttraction + ...
    speedControl + ...
    randomForce;