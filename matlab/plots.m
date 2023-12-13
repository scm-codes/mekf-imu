%% plots
% compute necessary quantities
time = 0:delta_t:(nsteps-1)*delta_t;
r2d = 180/pi;


% acceleration
figure;
subplot(3,1,1)
plot(time,accel_data ,'LineWidth',2);
xlabel('time [s]')
ylabel('acceleration [m/s^2]')
legend('ax','ay','az')
grid on

subplot(3,1,2)
plot(time,gyro_data*r2d ,'LineWidth',2);
xlabel('time [s]')
ylabel('angular rate [deg/s]')
legend('wx','wy','wz')
grid on

subplot(3,1,3)
plot(time,mag_data ,'LineWidth',2);
xlabel('time [s]')
ylabel('magnetic field [microT]')
legend('mx','my','mz')
grid on

%% Estimator Output
figure; 
subplot(2,2,1)
plot(time,[posterior_euler(:,3),posterior_euler(:,2),posterior_euler(:,1)]*r2d,'LineWidth',2);
hold on
rectangle('Position',[43 -10 6 15],'EdgeColor','k', 'LineWidth',2)
xlabel('time [s]')
ylabel('euler angles [deg]')
title('MEKF Estimate')
legend('roll','pitch','yaw')
grid on

subplot(2,2,2)
hold on
plot(time,[posterior_euler(:,3),posterior_euler(:,2),posterior_euler(:,1)]*r2d,'LineWidth',2);
xlim([43 49])
ylim([-10 5])
rectangle('Position',[43 -10 6 15],'EdgeColor','k', 'LineWidth',2)
daspect([1,1,1])
title('MEKF Estimate')
 
subplot(2,2,3)
plot(time,deadrec_euler(2:end,:)*r2d,'LineWidth',2);
hold on
rectangle('Position',[43 -10 6 15],'EdgeColor','k', 'LineWidth',2)
xlabel('time [s]')
ylabel('euler angles [deg]')
title('Dead Reckoning')
legend('roll','pitch','yaw')
grid on

subplot(2,2,4)
hold on
plot(time,deadrec_euler(2:end,:)*r2d,'LineWidth',2);
xlim([43 49])
ylim([-10 5])
rectangle('Position',[43 -10 6 15],'EdgeColor','k', 'LineWidth',2)
daspect([1,1,1])
title('Dead Reckoning')
