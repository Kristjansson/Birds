clear all; close all; clc;
n = norm([1,1,0]);
% [a,b,c] = fwdDirAndBeta2basis([1;0;0], 0)
[d,e,f] = fwdDirAndBeta2basis([1;1;0]/n, 0)
% [h,i,j] = fwdDirAndBeta2basis([0;1;0], 0)
[k,l,m] = fwdDirAndBeta2basis([-1;1;0]/n, 0)
% [n,o,p] = fwdDirAndBeta2basis([-1;0;0], 0)
[q,r,s] = fwdDirAndBeta2basis([-1;-1;0]/n, 0)
% [t,u,v] = fwdDirAndBeta2basis([0;-1;0], 0)
[w,x,y] = fwdDirAndBeta2basis([1;-1;0]/n, 0)