
clear all
close all
clc


u = tf(0.025,[0.75 1])
v = tf(0.1755,[10 1])
r = tf(0.0184,[0.75 1])

step(u,v,r)
legend