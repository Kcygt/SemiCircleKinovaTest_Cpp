close all
data = readmatrix("data.csv");

figure(1)
hold on; grid on;
plot(data(:,18),data(:,16))
plot(data(:,27),data(:,25))
xlabel('Z direction')
ylabel('X direction')
legend('Desired path','Actual path')


Ez30 = rmse(data(:,18),data(:,27));
Ex30 = rmse(data(:,16),data(:,25));