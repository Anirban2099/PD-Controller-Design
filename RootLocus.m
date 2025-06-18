clc;
close all;
clear all;
num=[0.05];
den=[1 0.35 0.625 0.075 0]
TF=tf(num,den)
rlocus(TF)
