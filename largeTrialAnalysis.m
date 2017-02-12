clear all; close all; clc;
load('largeTrial.mat', 'velOverTime', 'NumBirds', 'NumTimeSteps', 'bankingOverTime')

bird = 1;
fwdDirs = zeros(3, NumTimeSteps);
wingDirs = zeros(3, NumTimeSteps);
upDirs = zeros(3, NumTimeSteps);
fwdDirDotUpDir = zeros(NumTimeSteps, 1);
fwdDirDotWingDir = zeros(NumTimeSteps, 1);
wingDirDotUpDir = zeros(NumTimeSteps, 1);

for itr=1:NumTimeSteps
    fwdDirs(:, itr) = velOverTime(:, bird, itr)/norm(velOverTime(:, bird, itr));
    [wingDirs(:, itr), upDirs(:, itr)] = fwdDirAndBeta2basis(fwdDirs(:, itr), bankingOverTime(bird, itr));
    fwdDirDotUpDir(itr) = dot(fwdDirs(:, itr), upDirs(:, itr));
    fwdDirDotWingDir(itr) = dot(fwdDirs(:, itr), wingDirs(:, itr));
    wingDirDotUpDir(itr) = dot(wingDirs(:, itr), upDirs(:, itr));
end

%%

figure
subplot(3,1,1)
plot(1:NumTimeSteps, fwdDirDotUpDir); hold on;
subplot(3,1,2)
plot(1:NumTimeSteps, fwdDirDotWingDir); hold on;

subplot(3,1,3)
plot(1:NumTimeSteps, wingDirDotUpDir); hold on;

figure
subplot(3,1,1)
plot(1:NumTimeSteps, fwdDirs(1, :)); hold on;
subplot(3,1,2)
plot(1:NumTimeSteps, fwdDirs(2, :)); hold on;
subplot(3,1,3)
plot(1:NumTimeSteps, fwdDirs(3, :)); hold on;

figure
subplot(3,1,1)
plot(1:NumTimeSteps, wingDirs(1, :)); hold on;
subplot(3,1,2)
plot(1:NumTimeSteps, wingDirs(2, :)); hold on;
subplot(3,1,3)
plot(1:NumTimeSteps, wingDirs(3, :)); hold on;

figure
subplot(3,1,1)
plot(1:NumTimeSteps, upDirs(1, :)); hold on;
subplot(3,1,2)
plot(1:NumTimeSteps, upDirs(2, :)); hold on;
subplot(3,1,3)
plot(1:NumTimeSteps, upDirs(3, :)); hold on;


%%
figure
subplot(3,1,1)
plot(1:NumTimeSteps, sum(fwdDirs.^2)); hold on;
subplot(3,1,2)
plot(1:NumTimeSteps, sum(wingDirs.^2)); hold on;
subplot(3,1,3)
plot(1:NumTimeSteps, sum(upDirs.^2)); hold on;

%%
load('largeTrial.mat', 'posOverTime', 'X', 'Y', 'Z')

figure
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







