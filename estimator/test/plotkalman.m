%%% illustration of Kalman filtering

clear;
close all;

%% load data
kalman = csvread("data/kalman.csv");
observed = csvread("data/observed.csv");
truex = csvread("data/truex.csv");
Peigen=csvread("data/EigenP.csv");

%% 
figure(1)

subplot(211)
plot(kalman(1,:),'-ro'); hold on;
plot(observed(1,:),'-sb');hold on;
plot(truex(1,:),'-k.');
legend("Kalman","observed","true");

subplot(212)
plot(Peigen,'-ro'); 

