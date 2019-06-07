clear;
close all;

index_actuation=0; % underactuated=1

path = '../data/';
Balpha = csvread(strcat(path, 'Balpha.csv'),1,0);
u = csvread(strcat(path, 'u.csv'),1,0);
alpha = csvread(strcat(path, 'alpha.csv'),1,0);
alpha_deg = csvread(strcat(path, 'alpha_deg.csv'),1,0);
dtau = csvread(strcat(path, 'tau.csv'),1,0);
rotation = csvread(strcat(path, 'rotation.csv'),1,0);


m=size(alpha,2);

figure(1); 
subplot(311);
plot(Balpha(2:end,1), '-r', 'linewidth', 2);
hold on;
plot(dtau(:,1), ':k', 'linewidth', 2);
ylabel('taux(N)');
legend('estimated force', 'desired force');
subplot(312); 
plot(Balpha(2: end,2),'-r', 'linewidth', 2);
hold on;
plot(dtau(:,2), ':k', 'linewidth', 2);
ylabel('tauy(N)') 
legend('estimated force', 'desired force') 
subplot(313);
plot(Balpha(2:end,3), '-r', 'linewidth', 2);
hold on;
plot(dtau(:,3), ':k', 'linewidth', 2);
legend('estimated force', 'desired force');
ylabel('taun(N m)')

figure(2); 
for i=1:m
    subplot(m,1,i);
    plot(u(:,i),'linewidth', 2);
    ylabel(['u ',num2str(i)]);
end

    
figure(3);
for i=1:m
    subplot(m,1,i);
    plot(alpha(:,i) *180 / pi,'linewidth', 2); hold on;
    plot(alpha_deg(:,i), ':k', 'linewidth', 2);
    legend('double angle', 'int angle');
    ylabel(['alpha(deg) ',num2str(i)]);
end

figure(4);
for i=1:m
    subplot(m,1,i);
    plot(rotation(:,i),'linewidth', 2);
    ylabel(['rotation(rpm) ',num2str(i)]);
end

