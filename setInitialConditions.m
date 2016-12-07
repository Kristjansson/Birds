initialConditionSet = 7;

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
        NumTimeSteps = 5000;

        posOverTime = zeros(3, NumBirds, NumTimeSteps);
        velOverTime = zeros(3, NumBirds, NumTimeSteps);
        bankingOverTime = zeros(NumBirds, NumTimeSteps);
        interactionRadiusOverTime = zeros(NumBirds, NumTimeSteps);

%         posOverTime(:, 1:NumBirds, 1) = repmat([0;-1; z0], 1, NumBirds);
        posOverTime(:, 1:NumBirds, 1) = [2*rand(2, NumBirds, 1) - ones(2, NumBirds); z0*ones(1, NumBirds)];
        
%         velOverTime(:, 1:NumBirds, 1) = repmat(v0*[1; 0; 0], 1, NumBirds);
        velOverTime(:, 1:NumBirds, 1) = [v0*rand(2, NumBirds); zeros(1, NumBirds)];
        bankingOverTime(1:NumBirds, 1) = repmat([0], NumBirds, 1);
        interactionRadiusOverTime(1:NumBirds, 1:(du/dt)) = repmat([30], NumBirds, (du/dt));
    case 6
        % large scale test
        NumBirds = 2000;
        NumTimeSteps = 100;

        posOverTime = zeros(3, NumBirds, NumTimeSteps);
        velOverTime = zeros(3, NumBirds, NumTimeSteps);
        bankingOverTime = zeros(NumBirds, NumTimeSteps);
        interactionRadiusOverTime = zeros(NumBirds, NumTimeSteps);

%         posOverTime(:, 1:NumBirds, 1) = repmat([0;-1; z0], 1, NumBirds);
        posOverTime(:, 1:NumBirds, 1) = [20*(2*rand(2, NumBirds, 1) - ones(2, NumBirds)); z0*ones(1, NumBirds)];
        
%         velOverTime(:, 1:NumBirds, 1) = repmat(v0*[1; 0; 0], 1, NumBirds);
        velOverTime(:, 1:NumBirds, 1) = [v0*rand(2, NumBirds); zeros(1, NumBirds)];
        bankingOverTime(1:NumBirds, 1) = repmat([0], NumBirds, 1);
        interactionRadiusOverTime(1:NumBirds, 1:(du/dt)) = repmat([30], NumBirds, (du/dt));
    case 7
        NumBirds = 30;
        NumTimeSteps = 5000;

        posOverTime = zeros(3, NumBirds, NumTimeSteps);
        velOverTime = zeros(3, NumBirds, NumTimeSteps);
        bankingOverTime = zeros(NumBirds, NumTimeSteps);
        interactionRadiusOverTime = zeros(NumBirds, NumTimeSteps);

%         posOverTime(:, 1:NumBirds, 1) = repmat([0;-1; z0], 1, NumBirds);
        posOverTime(:, 1:NumBirds, 1) = [20*(2*rand(2, NumBirds, 1) - ones(2, NumBirds)); 30*rand(1, NumBirds) - 5 + z0];
        
%         velOverTime(:, 1:NumBirds, 1) = repmat(v0*[1; 0; 0], 1, NumBirds);
        velOverTime(:, 1:NumBirds, 1) = [v0*rand(2, NumBirds); zeros(1, NumBirds)];
        bankingOverTime(1:NumBirds, 1) = repmat([0], NumBirds, 1);
%         interactionRadiusOverTime(1:NumBirds, 1:(du/dt)) = repmat([30], NumBirds, (du/dt));
    case 8
        % attempt to generate gimble lock
        NumBirds = 20;
        NumTimeSteps = 5000;

        posOverTime = zeros(3, NumBirds, NumTimeSteps);
        velOverTime = zeros(3, NumBirds, NumTimeSteps);
        bankingOverTime = zeros(NumBirds, NumTimeSteps);
        interactionRadiusOverTime = zeros(NumBirds, NumTimeSteps);
        posOverTime(:, 1:NumBirds, 1) = [5*(2*rand(2, NumBirds, 1) - ones(2, NumBirds)); 0*rand(1, NumBirds) - 5 + z0];
        
        velOverTime(:, 1:NumBirds, 1) = [v0*rand(2, NumBirds); zeros(1, NumBirds)];
        bankingOverTime(1:NumBirds, 1) = repmat([0], NumBirds, 1);
end