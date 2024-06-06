close all;
clc;
clear;

%% Parameters
v = 1; %[m/s]
%%

A = [0 v;
    0 0];

B = [0;
    1];

Q_s = eye(2);
r = 20;
Q_t = eye(2); %not used since T -> oo


[K,S,P] = lqr(ss(A,B,zeros(2,2),0),Q_s,r);









