close all
data = readmatrix("data.csv");
x_FK = zeros(length(data(:,1)),3);
xp = zeros(length(data(:,1)),6);
q_IK = zeros(length(data(:,1)),7);
x_IK = zeros(length(data(:,1)),3);

for i =1:length(x_FK)
    x_FK(i,:) = forwardKinematics(data(i,2:8));
    xp(i,:) = jacobiann(data(i,2:8)) * data(i,9:15)';
    q_IK(i,:) = inverseKinematics(data(i,2:8),xp(i,:));
    x_IK(i,:) = forwardKinematics(q_IK(i,:));
end



figure(1)
hold on; grid on;
plot(data(:,18),data(:,16))
plot(data(:,27),data(:,25))
xlabel('Z direction')
ylabel('X direction')
legend('Desired path','Actual path')


figure(2)
hold on; grid on;
plot(x_FK(:,3),x_FK(:,1))
ylabel('x direction (cm)')
xlabel('z direction (cm)')
title(['xpd obtained by hypodrome function, put inverse kinematic to get' ...
    ' joint velocity and then multiply with time to get joint position'])

figure(3)
hold on; grid on;   
plot(data(:,2))