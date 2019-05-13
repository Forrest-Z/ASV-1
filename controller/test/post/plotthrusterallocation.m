clear;
close all;

path = '../data/';
Rtau = csvread(strcat(path, 'Balpha.csv'),1,0);
u = csvread(strcat(path, 'u.csv'),1,0);
alpha = csvread(strcat(path, 'alpha.csv'),1,0);
alpha_deg = csvread(strcat(path, 'alpha_deg.csv'),1,0);
dtau = csvread(strcat(path, 'tau.csv'),1,0);
rotation = csvread(strcat(path, 'rotation.csv'),1,0);

figure; 
subplot(311);
plot(Rtau(2:end,1), '-r', 'linewidth', 2);
hold on;
plot(dtau(:,1), ':k', 'linewidth', 2);
ylabel('taux(N)');
legend('achieved force', 'desired force');
subplot(312); 
plot(Rtau(2: end,2),'-r', 'linewidth', 2);
hold on;
plot(dtau(:,2), ':k', 'linewidth', 2);
ylabel('tauy(N)') 
legend('achieved force', 'desired force') 
subplot(313);
plot(Rtau(2:end,3), '-r', 'linewidth', 2);
hold on;
plot(dtau(:,3), ':k', 'linewidth', 2);
legend('achieved force', 'desired force');
ylabel('taun(N m)')

figure;
subplot(311);
plot(u(:,1),'linewidth', 2);
ylabel('u1(%)');
title('real-time thrust');
subplot(312);
plot(u(:,2),'linewidth', 2);
ylabel('u2(%)'); 
subplot(313);
plot(u(:,3), 'linewidth', 2);
ylabel('u3(%)')

    
figure;
subplot(311); 
plot(alpha(:,1) *180 / pi, 'linewidth', 2);
hold on;
plot(alpha_deg(:,1), ':k', 'linewidth', 2);
legend('double angle', 'int angle');
ylabel('a1(deg)');
title('real-time angle');
subplot(312);
plot(alpha(:,2)* 180 / pi, 'linewidth', 2);
hold on;
plot(alpha_deg(:,2), ':k', 'linewidth', 2);
legend('double angle', 'int angle');
ylabel('a2(deg)');
subplot(313);
plot(alpha(:,3) * 180 / pi, 'linewidth', 2);
hold on;
plot(alpha_deg(:,3), ':k', 'linewidth', 2);
legend('double angle', 'int angle');
ylabel('a3(deg)')


figure 
subplot(311);
plot(rotation(:,1),'linewidth', 2);
ylabel('a1(deg)');
title('real-time rotation');
subplot(312);
plot(rotation(:,2), 'linewidth', 2);
ylabel('a2(deg)');
subplot(313);
plot(rotation(:,3), 'linewidth', 2);
ylabel('a3(deg)');
