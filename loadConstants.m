dt = 0.005;
v0 = 10;
mass = 0.08;
gravity = 9.81;
CD_CL = 3.3;
L0 = mass * gravity;
T0 = CD_CL * L0;
Tau = 1;
z0 = 10;
Rroost = 150;
du = 0.05;
s = 0.005;
Rmax = 100;
nc = 6;
rh = 0.2;
sigma = 1.77;
rsep = 4;

wAlt = 0.05; % Make stronger? 0.2 => 0.8
WRoost = 0.5; %Adjusted from 0.01 to produce tighter orbits. 
wSigma = 0.01;
ws = 1;
wc = 1;
wa = 0.5;
wBin = 10;
wBout = 1;

wr = s/du;

% g = 9.81;  %m/s^2 gravity
% dt = 0.005;  %s      integration time step
% du = 50;  %ms        Reaction time !!check s if you change!!
% v0 = 0.01;  %m/ms    Cruise Speed
% mass = 0.08;  %kg    Mass
% CL_CD = 3.3;  %      Lift-drag coefficient
% L0 = 0.78;  % N      Default Lift
% D0 = 0.24;  %N       Default Drag
% T0 = CL_CD * mass * g;  %N       Default Thrust
% wBin = 10;  %        Banking Control
% wBout = 1;  %        Banking Control
% T = 62.5;  %s        Speed Control
% Rmax = 100;  %m      Maximum Percetion Radius
% nc = 6.5;  %         Topological Range
% s = 0.1;  %          Interpolation Factor !!50 is du!!
% rh = 0.2;  %m        Radius of Max Separation
% rsep = 4;  %m        Seperation Radius
% sigma = 1.37;  %m    Parameter of the Gaussian
% ws = 1;  %N          Weighting factor separation force
% blindAngle = pi/2;  %Rear "blind angle" cohesion and alignment
% wa = 0.5;  %N        Weighting factor alignment force
% wc = 1;  %N          Weighting factor cohesion force
% Cc = 0.35;  %        Critical Centrality 
% wSigma = 0.01;     %N   Weight factor random force
% RRoost = 150;  %m    Radius of Roost
% WRoostH = 0.01;  %N/m Weighting factor horizontal attraction to roost
% WRoostV = 0.2;  %N    Weighting factor vertical attraction to roost
% 
% %Convert Forces 
% ForceConversion = 1e-06; % 1 Newton => 1e-06; % kg m/(ms)^2
% 
% L0 = L0 * ForceConversion;
% D0 = D0 * ForceConversion;
% % T0 = T0 * ForceConversion;
% ws = ws * ForceConversion;
% wa = wa * ForceConversion;
% wc = wc * ForceConversion;
% wSigma = wSigma * ForceConversion;
% WRoostH = WRoostH * ForceConversion;
% WRoostV = WRoostV * ForceConversion;