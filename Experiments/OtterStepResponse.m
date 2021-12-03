
clear all
close all
clc


sysu = tf(0.025,[0.75 1])

sysv = tf(0.1755,[10 1])

sysr = tf(0.0184,[0.75 1])

step(sysu,sysv,sysr)