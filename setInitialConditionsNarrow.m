NumBirds = 30;
NumTimeSteps = 5000;
posOverTime = zeros(3, NumBirds, NumTimeSteps);
velOverTime = zeros(3, NumBirds, NumTimeSteps);
bankingOverTime = zeros(NumBirds, NumTimeSteps);
interactionRadiusOverTime = zeros(NumBirds, NumTimeSteps);
posOverTime(:, 1:NumBirds, 1) = [20*(2*rand(2, NumBirds, 1) - ones(2, NumBirds)); ...
	5*rand(1, NumBirds) - 5 + z0];
velOverTime(:, 1:NumBirds, 1) = [v0*rand(2, NumBirds); zeros(1, NumBirds)];