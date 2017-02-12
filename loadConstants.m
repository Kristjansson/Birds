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

liftConstant = L0/v0^2;
dragConstant = -CD_CL/v0^2 * mass * gravity;
gravityForce = -[0;0;mass*gravity];

wAlt = 0.2; 
wRoost = 0.1066666; %Adjusted from 0.01
wSigma = 0.01;
ws = 1;
wc = 1;
wa = 0.5;
wBin = 1;
wBout = 5;

wr = s/du;
