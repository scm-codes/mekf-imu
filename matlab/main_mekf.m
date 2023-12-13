%% Multiplicative Extended Kalman Filter 
% Estimate Attidue using IMU data
% Data is obtained from using phyphox application on Iphone
% Data is recorded at 100 Hz

clc; close all; clear all;

% states
%   [1:3] orientation error
%   [4:6] velocity error
%   [7:9] position error
%   [9:12] gyro bias
%   [13:15] accelerometer bias
%   [16:18] magnetometer bias

% inputs
%   [1:3] acceleration observations
%   [4:6] magnetometer observations

%% read imu data

% degree 2 radians
d2r = pi/180;
 
%%% load phyphox data 
accel_table = readtable("data/phyphox_01/Accelerometer.csv");
gyro_table = readtable("data/phyphox_01/Gyroscope.csv");
mag_table = readtable("data/phyphox_01/Magnetometer.csv");

% pre-process data
% gyro and mag have same time stamps but not accel
gyro_data = [gyro_table.X_rad_s_ gyro_table.Y_rad_s_ gyro_table.Z_rad_s_];
mag_data = [mag_table.X__T_ mag_table.Y__T_ mag_table.Z__T_];
accel_data = [accel_table.X_m_s_2_ accel_table.Y_m_s_2_ accel_table.Z_m_s_2_];
accel_data = interp1(accel_table.Time_s_, accel_data, gyro_table.Time_s_,"nearest","extrap");

nsteps = length(accel_data);
delta_t = 0.01;

%% inputs to filter

% number of states
global nstates vecdim g_ned mag_ned
nstates = 18;
vecdim = 3;

% I have used the average of first few observations as reference vector
g_ned = [-0.0251;    0.0034;    9.8446]; 
mag_ned = [-22.6038; 0.6232; -50.7185];  

% define initial conditions
initial_estimate = [1 0 0 0];
initial_estimate_cov = 10*eye(nstates);

% define inital bias 
gyro_bias = [0; 0; 0];
accel_bias = [0; 0; 0];
mag_bias = [0; 0; 0];

% define variance
% these quantities control the performance of the filter
gyro_var = 0.05;
gyro_bias_var = 0.001;
accel_var = 0.05;
accel_bias_var = 0.001;
accel_obsv_var = 0.5;
mag_var = 0.05;
mag_bias_var = 0.001;
mag_obsv_var = 0.5;

% define covariance matrices
gyro_cov_mat = gyro_var*eye(vecdim);
gyro_bias_cov_mat = gyro_bias_var*eye(vecdim);
accel_cov_mat = accel_var*eye(vecdim);
accel_bias_cov_mat = accel_bias_var*eye(vecdim);
mag_bias_cov_mat = mag_bias_var*eye(vecdim);
obsv_cov_mat = blkdiag(accel_obsv_var*eye(vecdim), mag_obsv_var*eye(vecdim));

%% initialize filter
estimate = initial_estimate;
estimate_cov = initial_estimate_cov;

% NOTE: requires omega 
posterior_estimate = zeros(nsteps,length(estimate));
posterior_euler = zeros(nsteps,3);

% define deadreckoning stuff
deadrec_euler = zeros(nsteps+1,3);

for ii = 1:nsteps
    
    % gyro meas
    gyro_meas = gyro_data(ii,:)';
    accel_meas = accel_data(ii,:)';
    mag_meas = mag_data(ii,:)';
    
     %%%% Dead Reckoning
    deadrec_eulerdot = deadrec_fun(deadrec_euler(ii,:), gyro_meas);
    deadrec_euler(ii+1,:) = deadrec_euler(ii,:)' + deadrec_eulerdot*delta_t;

    % correct measurements 
    gyro_meas = gyro_meas - gyro_bias;
    accel_meas = accel_meas - accel_bias;
    mag_meas = mag_meas - mag_bias;
    
    % integrate angular velocity using qarternion derivative
    estimate = estimate + 0.5*delta_t*quatmultiply(estimate, [0 gyro_meas']);
    estimate = quatnormalize(estimate);
    
    % update the process model
    Phi = get_Phi(estimate, gyro_meas, accel_meas, delta_t);
    
    % get process matrix
    Q = get_Q(delta_t, gyro_cov_mat, gyro_bias_cov_mat,...
        accel_cov_mat, accel_bias_cov_mat,...
        mag_bias_cov_mat);
 
    % update the priori covariance
    estimate_cov = Phi*estimate_cov*Phi' + Q;
    
    % get H matrix
    H = get_H(estimate);
    
    % compute kalman gain
    K = estimate_cov*H'/(H*estimate_cov*H' + obsv_cov_mat);
    
    % caompute error at each step
     err = [ accel_meas - quat2dcm(estimate)*g_ned; ...
               mag_meas - quat2dcm(estimate)*mag_ned];
    
    % update posetior state estimate 
    posterior_state = K*err;
    
    % update posterior covariance
    estimate_cov = (eye(nstates) - K*H)*estimate_cov;
    
    % update the estimate
    estimate = quatmultiply(estimate, [1 0.5*posterior_state(1:3)']);
    estimate = quatnormalize(estimate);
    
    % update all bias
    gyro_bias = gyro_bias + posterior_state(10:12);
    accel_bias = accel_bias + posterior_state(13:15);
    mag_bias = mag_bias + posterior_state(16:18);
    
    % estimates
    posterior_estimate(ii,:) = estimate;
    posterior_euler(ii,:) = quat2eul(estimate);
    
end

plots; 
animate;

%% support functions

function Q = get_Q(delta_t, ...
            gyro_cov_mat, gyro_bias_cov_mat, ...
            accel_cov_mat, accel_bias_cov_mat, ...
            mag_bias_cov_mat)
    % builds the process covariance
    
    global nstates
    
    Q = zeros(nstates, nstates);
    Q(1:3, 1:3) = gyro_cov_mat*delta_t + gyro_bias_cov_mat*delta_t^3/3;
    Q(1:3, 10:12) = -gyro_bias_cov_mat*delta_t^2/2;
    
    Q(4:6, 4:6) = accel_cov_mat*delta_t + accel_bias_cov_mat*delta_t^3/3;
    Q(4:6, 7:9) = accel_bias_cov_mat*delta_t^4/8 + accel_cov_mat*delta_t^2/2;
    Q(4:6, 13:15) = -accel_bias_cov_mat*delta_t^2/2;
    
    Q(7:9, 4:6) = accel_cov_mat*delta_t^2/2 + accel_bias_cov_mat*delta_t^4/8;
    Q(7:9, 7:9) = accel_cov_mat*delta_t^3/3 + accel_bias_cov_mat*delta_t^5/20;
    Q(7:9, 13:15) = -accel_bias_cov_mat*delta_t^3/6;
    
    Q(10:12, 1:3) = -gyro_bias_cov_mat*delta_t^2/2;
    Q(10:12, 10:12) = gyro_bias_cov_mat*delta_t;
    
    Q(13:15, 4:6) = -accel_bias_cov_mat*delta_t^2/2;
    Q(13:15, 7:9) = -accel_bias_cov_mat*delta_t^3/6;
    Q(13:15, 13:15) = accel_bias_cov_mat*delta_t;
    
    Q(16:18, 16:18) = mag_bias_cov_mat*delta_t;
        
end

function Phi = get_Phi(estimate, gyro_meas, accel_meas, delta_t)
    % builds the discrete time dynamics matrix
    
    global nstates vecdim
    
    F = zeros(nstates, nstates);
    F(1:3, 1:3) = -skew_symmetric(gyro_meas);
    F(1:3, 10:12) = -eye(vecdim);
    F(4:6, 1:3) = -quat2rotm(estimate)*skew_symmetric(accel_meas);
    F(4:6, 13:15) = -quat2rotm(estimate);
    F(7:9, 4:6) = eye(vecdim);
    
    Phi = eye(nstates) + F*delta_t;
    
end


function H = get_H(estimate)
    % builds the H matrix
    
    global nstates vecdim g_ned mag_ned
    
    H = zeros(vecdim, nstates);
    H(1:3, 1:3) = skew_symmetric(quat2dcm(estimate)*g_ned);
    H(1:3, 13:15) = eye(vecdim);
    H(4:6, 1:3) = skew_symmetric(quat2dcm(estimate)*mag_ned);
    H(4:6, 16:18) = eye(vecdim);
    
end

function mat = skew_symmetric(vec)
    % builds skew symmetric matrix 
    
    mat = [0 -vec(3) vec(2);
           vec(3) 0 -vec(1);
           -vec(2) vec(1) 0];

end

function deadrec_euldot = deadrec_fun(euler, angular_rate)
    % computes euler angles rate from angular rate
    
    % unpack
    roll = euler(1);
    pitch = euler(2);
    yaw = euler(3);
    
    % poissons euqation
    deadrec_euldot = 1/cos(pitch)*[cos(pitch) sin(roll)*sin(pitch) cos(roll)*sin(pitch); ...
                                   0 cos(roll)*cos(pitch) -sin(roll)*cos(pitch); ...
                                   0 sin(roll) cos(roll)]*angular_rate;

end 
