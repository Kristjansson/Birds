initialConditionSet = 5;

% posOverTime(:, 1, 1) = [20;1;10];
% posOverTime(:, 2, 1) = [-20;1;10];
% % velOverTime(:, 1, 1) = v0*[1;0;1]/norm([1;0;1]);
% % velOverTime(:, 2, 1) = (v0 - 0.009)*[1;0;0];
% temp1 = [1;0;0]
% temp2 = [0;1;0]
% velOverTime(:, 1, 1) = v0*temp1 / norm(temp1);
% velOverTime(:, 2, 1) = 2*v0*temp2 / norm(temp2);
% % velOverTime(:, 1, 1) = [1;1;1];
% % should result in ex = [1 1 1], ey = [1 -1 0], ez = [-1 -1 2]
% bankingOverTime(1, 1) = 0;
% bankingOverTime(2, 1) = 0;
% interactionRadiusOverTime(1:NumBirds, 1:(du/dt)) = Rmax * ones(NumBirds, (du/dt));

switch initialConditionSet
    case 1
        NumBirds = 20;
        NumTimeSteps = 1000;

        posOverTime = zeros(3, NumBirds, NumTimeSteps);
        velOverTime = zeros(3, NumBirds, NumTimeSteps);
        bankingOverTime = zeros(NumBirds, NumTimeSteps);
        interactionRadiusOverTime = zeros(NumBirds, NumTimeSteps);

        posOverTime(1:2, 1:NumBirds, 1) = 10*rand(2, NumBirds, 1);
        posOverTime(3, 1:NumBirds, 1) = 0.5*rand(1, NumBirds, 1) + z0 - 0.5;

        velOverTime(:, 1:NumBirds, 1) = [2*v0*rand(2, NumBirds, 1) - 10; zeros(1, NumBirds)];
        bankingOverTime(1:NumBirds, 1) = zeros(NumBirds, 1);
        interactionRadiusOverTime(1:NumBirds, 1:(du/dt)) = Rmax * ones(NumBirds, (du/dt));
    case 2
        % dipping when slowing test
        NumBirds = 1;
        NumTimeSteps = 5000;

        posOverTime = zeros(3, NumBirds, NumTimeSteps);
        velOverTime = zeros(3, NumBirds, NumTimeSteps);
        bankingOverTime = zeros(NumBirds, NumTimeSteps);
        interactionRadiusOverTime = zeros(NumBirds, NumTimeSteps);

        posOverTime(:, 1:NumBirds, 1) = [0;-1; z0];
 
        velOverTime(:, 1:NumBirds, 1) = 0.5*v0*[1; 0; 0];
        bankingOverTime(1:NumBirds, 1) = 0;
        interactionRadiusOverTime(1:NumBirds, 1:(du/dt)) = Rmax;
    case 3
        % altitude control test
        NumBirds = 1;
        NumTimeSteps = 5000;

        posOverTime = zeros(3, NumBirds, NumTimeSteps);
        velOverTime = zeros(3, NumBirds, NumTimeSteps);
        bankingOverTime = zeros(NumBirds, NumTimeSteps);
        interactionRadiusOverTime = zeros(NumBirds, NumTimeSteps);

        posOverTime(:, 1:NumBirds, 1) = [0;-1; 1.5*z0];
 
        velOverTime(:, 1:NumBirds, 1) = v0*[1; 0; 0];
        bankingOverTime(1:NumBirds, 1) = 0;
        interactionRadiusOverTime(1:NumBirds, 1:(du/dt)) = Rmax;
    case 4
        % banking test
        NumBirds = 1;
        NumTimeSteps = 5000;

        posOverTime = zeros(3, NumBirds, NumTimeSteps);
        velOverTime = zeros(3, NumBirds, NumTimeSteps);
        bankingOverTime = zeros(NumBirds, NumTimeSteps);
        interactionRadiusOverTime = zeros(NumBirds, NumTimeSteps);

        posOverTime(:, 1:NumBirds, 1) = [0;-1; z0];
 
        velOverTime(:, 1:NumBirds, 1) = v0*[1; 0; 0];
        bankingOverTime(1:NumBirds, 1) = pi/4;
        interactionRadiusOverTime(1:NumBirds, 1:(du/dt)) = Rmax;
    case 5
        % current
        NumBirds = 20;
        NumTimeSteps = 1000;

        posOverTime = zeros(3, NumBirds, NumTimeSteps);
        velOverTime = zeros(3, NumBirds, NumTimeSteps);
        bankingOverTime = zeros(NumBirds, NumTimeSteps);
        interactionRadiusOverTime = zeros(NumBirds, NumTimeSteps);

        posOverTime(:, 1:NumBirds, 1) = repmat([0;-1; z0], 1, NumBirds);
 
        velOverTime(:, 1:NumBirds, 1) = repmat(v0*[1; 0; 0], 1, NumBirds);
        bankingOverTime(1:NumBirds, 1) = repmat([0], NumBirds, 1);
        interactionRadiusOverTime(1:NumBirds, 1:(du/dt)) = repmat([Rmax-1], NumBirds, (du/dt));
end