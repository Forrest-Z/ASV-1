clear;
close all;

Path='20190606/test1/';
load(strcat(Path,'controller.csv'));

%% generate timestamp in seconds (I vessel)
timestamp_first=(totaldata_first(:,3)-timestamp0)*86400.0;  % seconds
index_select_first=find(timeselect(1)<=timestamp_first & timestamp_first<= timeselect(2));
timestamp_first=timestamp_first(index_select_first);
Position_first=totaldata_first(index_select_first,4:9);
% setpoint
set_theta_first=-90; % deg
if (abs(set_theta_first)<=180 && abs(set_theta_first)>90 )
    index_negative=find(Position_first(:,6)<0);
    Position_first(index_negative,6)=Position_first(index_negative,6)+360;
end

State_first=totaldata_first(index_select_first,10:15);
Tau_first=totaldata_first(index_select_first,16:18);
est_first=totaldata_first(index_select_first,19:21);
alpha_first=totaldata_first(index_select_first,22:24);
rpm_first=totaldata_first(index_select_first,25:27);
State4control_first=totaldata_first(index_select_first,28:30);

firstvessel.timestamp=timestamp_first;
firstvessel.position=Position_first;
firstvessel.state=State_first;
firstvessel.tau=Tau_first;
firstvessel.est=est_first;
firstvessel.alpha=alpha_first;
firstvessel.rpm=rpm_first;
firstvessel.state4control=State4control_first;

name_second='Second.csv';
path_second=strcat(path,name_second);
totaldata_second=csvread(path_second,1,0);
% generate timestamp in seconds
timestamp_second=(totaldata_second(:,3)-timestamp0)*86400.0;  % seconds
index_select_second=find(timeselect(1)<=timestamp_second & timestamp_second<= timeselect(2));
timestamp_second=timestamp_second(index_select_second);
Position_second=totaldata_second(index_select_second,4:9);

set_theta_second=-90; % deg
if (abs(set_theta_second)<=180 && abs(set_theta_second)>90 )
    index_negative=find(Position_second(:,6)<0);
    Position_second(index_negative,6)=Position_second(index_negative,6)+360;
end

State_second=totaldata_second(index_select_second,10:15);
Tau_second=totaldata_second(index_select_second,16:18);
est_second=totaldata_second(index_select_second,19:21);
alpha_second=totaldata_second(index_select_second,22:24);
rpm_second=totaldata_second(index_select_second,25:27);
State4control_second=totaldata_second(index_select_second,28:30);

secondvessel.timestamp=timestamp_second;
secondvessel.position=Position_second;
secondvessel.state=State_second;
secondvessel.tau=Tau_second;
secondvessel.est=est_second;
secondvessel.alpha=alpha_second;
secondvessel.rpm=rpm_second;
secondvessel.state4control=State4control_second;


name_third='Third.csv';
path_third=strcat(path,name_third);
totaldata_third=csvread(path_third,1,0);
% generate timestamp in seconds
timestamp_third=(totaldata_third(:,3)-timestamp0)*86400.0;  % seconds
index_select_third=find(timeselect(1)<=timestamp_third & timestamp_third<= timeselect(2));
timestamp_third=timestamp_third(index_select_third);
Position_third=totaldata_third(index_select_third,4:9);

set_theta_third=90; % deg
if (abs(set_theta_third)<=180 && abs(set_theta_third)>90 )
    index_negative=find(Position_third(:,6)<0);
    Position_third(index_negative,6)=Position_third(index_negative,6)+360;
end

State_third=totaldata_third(index_select_third,10:15);
Tau_third=totaldata_third(index_select_third,16:18);
est_third=totaldata_third(index_select_third,19:21);
alpha_third=totaldata_third(index_select_third,22:24);
rpm_third=totaldata_third(index_select_third,25:27);
State4control_third=totaldata_third(index_select_third,28:30);


thirdvessel.timestamp=timestamp_third;
thirdvessel.position=Position_third;
thirdvessel.state=State_third;
thirdvessel.tau=Tau_third;
thirdvessel.est=est_third;
thirdvessel.alpha=alpha_third;
thirdvessel.rpm=rpm_third;
thirdvessel.state4control=State4control_third;

%% figure 1 of I vessel

figure(1)
subplot(311)
plot(firstvessel.timestamp,firstvessel.position(:,1),'-r','linewidth',2);hold on;
plot(firstvessel.timestamp,firstvessel.state(:,1),':k','linewidth',2); 
xlim(timeselect);
% ylim([0 0.1]);
legend('position','State');
ylabel('surge(m)');
title('Position comparison --- I vessel')


subplot(312)
plot(firstvessel.timestamp,firstvessel.position(:,2),'-r','linewidth',2); hold on;
plot(firstvessel.timestamp,firstvessel.state(:,2),':k','linewidth',2);
xlim(timeselect);
legend('position','State');
ylabel('sway(m)');

subplot(313)
plot(firstvessel.timestamp,firstvessel.position(:,6),'-r','linewidth',2);hold on;
plot(firstvessel.timestamp,firstvessel.state(:,3)*180/pi,':k','linewidth',2); 
xlim(timeselect);
legend('position','State');
xlabel('time(s)');
ylabel('Yaw(deg)');

%% figure 2 
figure(2)
subplot(311)
plot(firstvessel.timestamp,firstvessel.state(:,4),'-r','linewidth',2);
xlim(timeselect);
ylabel('surge(m/s)');
title('Velocity --- I vessel')

subplot(312)
plot(firstvessel.timestamp,firstvessel.state(:,5),'-r','linewidth',2);
xlim(timeselect);
ylabel('sway(m/s)');

subplot(313)
plot(firstvessel.timestamp,firstvessel.state(:,6),'-r','linewidth',2);
xlim(timeselect);
xlabel('time(s)');
ylabel('yaw(rad/s)');



%% figure 3
figure(3)
plot(firstvessel.position(:,2),firstvessel.position(:,1),'ro','MarkerSize',2); 
title('Trajectory --- I vessel');

%% figure 4
figure(4)
subplot(311)
plot(firstvessel.timestamp, firstvessel.tau(:,1),'-r','linewidth',2); hold on; 
plot(firstvessel.timestamp, firstvessel.est(:,1),':k','linewidth',2);
xlim(timeselect);
ylabel('surge(N)');
legend('Desired force','Estimated force');
title('PID force --- I vessel')
subplot(312)
plot(firstvessel.timestamp, firstvessel.tau(:,2),'-r','linewidth',2); hold on; 
plot(firstvessel.timestamp, firstvessel.est(:,2),':k','linewidth',2);
xlim(timeselect);
ylabel('sway(N)');
legend('Desired force','Estimated force');
subplot(313)
plot(firstvessel.timestamp, firstvessel.tau(:,3),'-r','linewidth',2); hold on; 
plot(firstvessel.timestamp, firstvessel.est(:,3),':k','linewidth',2);
xlim(timeselect);
xlabel('time(s)');
ylabel('Yaw(N*m)');
legend('Desired force','Estimated force');


%% figures of II vessel

figure(5)
subplot(311)
plot(secondvessel.timestamp,secondvessel.position(:,1),'-r','linewidth',2);hold on;
plot(secondvessel.timestamp,secondvessel.state(:,1),':k','linewidth',2); 
xlim(timeselect);
% ylim([0 0.1]);
legend('position','State');
ylabel('surge(m)');
title('Position comparison --- II vessel')

subplot(312)
plot(secondvessel.timestamp,secondvessel.position(:,2),'-r','linewidth',2); hold on;
plot(secondvessel.timestamp,secondvessel.state(:,2),':k','linewidth',2);
xlim(timeselect);
legend('position','State');
ylabel('sway(m)');

subplot(313)
plot(secondvessel.timestamp,secondvessel.position(:,6),'-r','linewidth',2);hold on;
plot(secondvessel.timestamp,secondvessel.state(:,3)*180/pi,':k','linewidth',2); 
xlim(timeselect);
legend('position','State');
xlabel('time(s)');
ylabel('Yaw(deg)');

%% figure 6 
figure(6)
subplot(311)
plot(secondvessel.timestamp,secondvessel.state(:,4),'-r','linewidth',2);
xlim(timeselect);
ylabel('surge(m/s)');
title('Velocity --- II vessel')

subplot(312)
plot(secondvessel.timestamp,secondvessel.state(:,5),'-r','linewidth',2);
xlim(timeselect);
ylabel('sway(m/s)');

subplot(313)
plot(secondvessel.timestamp,secondvessel.state(:,6),'-r','linewidth',2);
xlim(timeselect);
xlabel('time(s)');
ylabel('yaw(rad/s)');



%% figure 7
figure(7)
plot(secondvessel.position(:,2),secondvessel.position(:,1),'ro','MarkerSize',2); 
title('Trajectory --- II vessel');

%% figure 8
figure(8)
subplot(311)
plot(secondvessel.timestamp, secondvessel.tau(:,1),'-r','linewidth',2); hold on; 
plot(secondvessel.timestamp, secondvessel.est(:,1),':k','linewidth',2);
xlim(timeselect);
ylabel('surge(N)');
legend('Desired force','Estimated force');
title('PID force --- II vessel')
subplot(312)
plot(secondvessel.timestamp, secondvessel.tau(:,2),'-r','linewidth',2); hold on; 
plot(secondvessel.timestamp, secondvessel.est(:,2),':k','linewidth',2);
xlim(timeselect);
ylabel('sway(N)');
legend('Desired force','Estimated force');
subplot(313)
plot(secondvessel.timestamp, secondvessel.tau(:,3),'-r','linewidth',2); hold on; 
plot(secondvessel.timestamp, secondvessel.est(:,3),':k','linewidth',2);
xlim(timeselect);
xlabel('time(s)');
ylabel('Yaw(N*m)');
legend('Desired force','Estimated force');


%% figures of III vessel
figure(9)
subplot(311)
plot(thirdvessel.timestamp,thirdvessel.position(:,1),'-r','linewidth',2);hold on;
plot(thirdvessel.timestamp,thirdvessel.state(:,1),':k','linewidth',2); 
xlim(timeselect);
% ylim([0 0.1]);
legend('position','State');
ylabel('surge(m)');
title('Position comparison --- III vessel')

subplot(312)
plot(thirdvessel.timestamp,thirdvessel.position(:,2),'-r','linewidth',2); hold on;
plot(thirdvessel.timestamp,thirdvessel.state(:,2),':k','linewidth',2);
xlim(timeselect);
legend('position','State');
ylabel('sway(m)');

subplot(313)
plot(thirdvessel.timestamp,thirdvessel.position(:,6),'-r','linewidth',2);hold on;
plot(thirdvessel.timestamp,thirdvessel.state(:,3)*180/pi,':k','linewidth',2); 
xlim(timeselect);
legend('position','State');
xlabel('time(s)');
ylabel('Yaw(deg)');

%% figure 10 
figure(10)
subplot(311)
plot(thirdvessel.timestamp,thirdvessel.state(:,4),'-r','linewidth',2);
xlim(timeselect);
ylabel('surge(m/s)');
title('Velocity --- III vessel')

subplot(312)
plot(thirdvessel.timestamp,thirdvessel.state(:,5),'-r','linewidth',2);
xlim(timeselect);
ylabel('sway(m/s)');

subplot(313)
plot(thirdvessel.timestamp,thirdvessel.state(:,6),'-r','linewidth',2);
xlim(timeselect);
xlabel('time(s)');
ylabel('yaw(rad/s)');


%% figure 11
figure(11)
plot(thirdvessel.position(:,2),thirdvessel.position(:,1),'ro','MarkerSize',2); 
title('Trajectory --- III vessel');

%% figure 12
figure(12)
subplot(311)
plot(thirdvessel.timestamp, thirdvessel.tau(:,1),'-r','linewidth',2); hold on; 
plot(thirdvessel.timestamp, thirdvessel.est(:,1),':k','linewidth',2);
xlim(timeselect);
ylabel('surge(N)');
legend('Desired force','Estimated force');
title('PID force --- III vessel')
subplot(312)
plot(thirdvessel.timestamp, thirdvessel.tau(:,2),'-r','linewidth',2); hold on; 
plot(thirdvessel.timestamp, thirdvessel.est(:,2),':k','linewidth',2);
xlim(timeselect);
ylabel('sway(N)');
legend('Desired force','Estimated force');
subplot(313)
plot(thirdvessel.timestamp, thirdvessel.tau(:,3),'-r','linewidth',2); hold on; 
plot(thirdvessel.timestamp, thirdvessel.est(:,3),':k','linewidth',2);
xlim(timeselect);
xlabel('time(s)');
ylabel('Yaw(N*m)');
legend('Desired force','Estimated force');

%% figures of DMU load
figure(13)
subplot(311)
plot(DMU.timestamp,DMU.Load(:,1),'-r','linewidth',2);
xlim(timeselect);
% ylim([0 0.1]);
ylabel('A_x(kg)');
title('DMU load')

subplot(312)
plot(DMU.timestamp,DMU.Load(:,2),'-r','linewidth',2);
xlim(timeselect);
ylabel('A_y(kg)');

subplot(313)
plot(DMU.timestamp,DMU.Load(:,3),'-r','linewidth',2);
xlim(timeselect);
xlabel('time(s)');
ylabel('A_z(kg)')


save([case_name,'/dpdata.mat'],'firstvessel','secondvessel','thirdvessel','DMU');
   