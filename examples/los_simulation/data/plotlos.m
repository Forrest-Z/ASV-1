clear;
close all;

index_actuation=1; % underactuated=1
num_thruster = 1;
dim_control = 3;

path = './';
tcontroller = csvread(strcat(path, 'controller.csv'),1,0);
testimator = csvread(strcat(path, 'estimator.csv'),1,0);
% GPS = csvread(strcat(path, 'GPS.csv'),1,0);
tplanner = csvread(strcat(path, 'planner.csv'),1,0);

controller.timestamp=tcontroller(:,2);
controller.tau=tcontroller(:,3:(dim_control+2));
controller.alpha=tcontroller(:,(dim_control+3):(dim_control+num_thruster+2));
controller.rpm=tcontroller(:,(dim_control+num_thruster+3):(dim_control+2*num_thruster+2));
controller.est=tcontroller(:,(dim_control+2*num_thruster+3):(2*dim_control+2*num_thruster+2));

estimator.timestamp=testimator(:,2);
estimator.measurement=testimator(:,3:8);
estimator.State=testimator(:,9:14);
estimator.Perror=testimator(:,15:17);
estimator.Verror=testimator(:,18:20);

planner.timestamp=tplanner(:,2);
planner.setpoint=tplanner(:,3:5);
planner.vsetpoint=tplanner(:,6:8);
planner.command=tplanner(:,9:11);
planner.waypoints0=tplanner(:,12:13);
planner.waypoints1=tplanner(:,14:15);

time0=min(planner.timestamp(1),controller.timestamp(1));


controller.timestamp=(controller.timestamp-time0)*86400.0; % second
estimator.timestamp=(estimator.timestamp-time0)*86400.0; % second
planner.timestamp=(planner.timestamp-time0)*86400.0; % second

figure(1); 
subplot(311);
plot(controller.timestamp,controller.est(:,1), '-r', 'linewidth', 2);hold on;
plot(controller.timestamp,controller.tau(:,1), ':k', 'linewidth', 2);
ylabel('taux(N)');
legend('estimated force', 'desired force');
subplot(312); 
plot(controller.timestamp,controller.est(:,2), '-r', 'linewidth', 2);hold on;
plot(controller.timestamp,controller.tau(:,2), ':k', 'linewidth', 2);
ylabel('tauy(N)') 
legend('estimated force', 'desired force');
subplot(313);
plot(controller.timestamp,controller.est(:,3), '-r', 'linewidth', 2);hold on;
plot(controller.timestamp,controller.tau(:,3), ':k', 'linewidth', 2);
legend('estimated force', 'desired force');
ylabel('taun(N m)')

figure(2); 
for i=1:num_thruster
    subplot(num_thruster,1,i);
    plot(controller.timestamp,controller.rpm(:,i),'linewidth', 2);
    ylabel(['rpm ',num2str(i)]);
end

    
figure(3);
for i=1:num_thruster
    subplot(num_thruster,1,i);
    plot(controller.timestamp,controller.alpha(:,i),'linewidth', 2);
    ylabel(['alpha(deg) ',num2str(i)]);
end


figure(4);
subplot(321);
plot(estimator.timestamp,estimator.State(:,1),'linewidth', 2); hold on;
plot(planner.timestamp,planner.setpoint(:,1),'linewidth', 2);
legend('State', 'setpoint');
ylabel('x');
subplot(322);
plot(estimator.timestamp,estimator.State(:,2),'linewidth', 2); hold on;
plot(planner.timestamp,planner.setpoint(:,2),'linewidth', 2);
legend('State', 'setpoint');
ylabel('y');
subplot(323);
plot(estimator.timestamp,estimator.State(:,3),'linewidth', 2); hold on;
plot(planner.timestamp,planner.setpoint(:,3),'linewidth', 2);
legend('State', 'setpoint');
ylabel('theta');
subplot(324);
plot(estimator.timestamp,estimator.State(:,4),'linewidth', 2);
ylabel('u');
subplot(325);
plot(estimator.timestamp,estimator.State(:,5),'linewidth', 2);
ylabel('v');
subplot(326);
plot(estimator.timestamp,estimator.State(:,6),'linewidth', 2);
ylabel('r');






figure(5)
plot(planner.waypoints0(:,2), planner.waypoints0(:,1),'r*');hold on;
plot(planner.waypoints1(:,2), planner.waypoints1(:,1),'r*');hold on;
plot(estimator.State(:,2), estimator.State(:,1),'k.');hold on;
xlabel('East[m]');
ylabel('North[m]');
axis equal;
grid on;



figure(6);
subplot(311);
plot(estimator.timestamp,estimator.Perror(:,1),'linewidth', 2);
ylabel('x-error');
subplot(312);
plot(estimator.timestamp,estimator.Perror(:,2),'linewidth', 2);
ylabel('y-error');
subplot(313);
plot(estimator.timestamp,estimator.Perror(:,3),'linewidth', 2);
ylabel('theta-error');


figure(7);
subplot(311);
plot(estimator.timestamp,estimator.Verror(:,1),'linewidth', 2);
ylabel('U-error');
subplot(312);
plot(estimator.timestamp,estimator.Verror(:,2),'linewidth', 2);
ylabel('V-error');
subplot(313);
plot(estimator.timestamp,estimator.Verror(:,3),'linewidth', 2);
ylabel('R-error');


