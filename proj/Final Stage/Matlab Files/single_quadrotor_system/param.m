%% param.m
%%
%% Parameters for the quadrotor.
%%
%% Modified:
%%    14/08/10 - Liang Sun
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all;
clc;

P.F_mag = 4;

% sample rate of controller
P.samples_per_second = 30; %500;
P.Ts = 1/P.samples_per_second;

% frame rate for camera (make it a multiple of sample rate)
P.frames_per_second = P.samples_per_second/10;
P.Tcam = 1/P.frames_per_second;
% number of control samples camera is delayed
P.cam_delay = round(P.samples_per_second/P.frames_per_second);


P.height_d = 1;  % height of quadrotor trajectory above ground (m)

P.orbit.radius = 3;
P.quadrotor.V = 1;

% initial state
P.x0 = [...
    0;...          % pn:    north position (m)
    0;...          % pe:    east position (m)
    0;...          % pd:    position down (negative of altitude) (m)
    0;...          % u:     velocity along body x-axis (m/s)
    0;...          % v:     velocity along body y-axis (m/s)
    0;...          % w:     velocity along body z-axis (m/s)
    0*pi/180;...   % phi:   roll angle
    0*pi/180;...   % theta: pitch angle
    0*pi/180;...   % psi:   yaw angle
    0;...          % p:     roll rate
    0;...          % q:     pitch rate
    0;...          % r:     yaw rate
    ];


% mass and inertia
% inertia in kg-m^2
P.Jx = 0.057600;
P.Jy = 0.057600; %0.057600;
P.Jz = 0.087600;
% gravity (m/s^2)
P.g = 9.806650;
% mass (kg)
P.m = 1.56;

% airframe physical parameters
P.l    = 0.3;       % wingspan (m)
P.rho  = 1.268200;  % air density

% propeller characteristics
P.kmotor_thrust = 5.45;  % F = 5.45*delta, where delta\in[0,1]
P.kmotor_torque = 0.0549; %

% bias parameters for sensors
P.gyro_x_bias  = .025;
P.gyro_y_bias  = .025;
P.gyro_z_bias  = .025;
P.accel_x_bias = .025;
P.accel_y_bias = .025;
P.accel_z_bias = .025;

% noise parameters for sensors
P.sigma_gyro_x = 0.005;   % rad/sec
P.sigma_gyro_y = 0.005;   % rad/sec
P.sigma_gyro_z = 0.005;   % rad/sec
P.sigma_accel_x = 0.005;  % m/sec
P.sigma_accel_y = 0.005;  % m/sec
P.sigma_accel_z = 0.005;  % m/sec
P.sigma_altimeter = 0.02;  % m, ultrasonic ultimeter

% estimator gains
P.lpf_gyro = 100;  % low pass filter constant for the gyros (lpf/(s+lpf))
P.Ts_attitude = 0.05; % sample rate for attitude estimator
P.Q = diag([10; 10; 10; 1; 1; 1; .5; .5; .5]);
P.R_altimeter = 10*P.sigma_altimeter^2;
P.R_pixel = .1;
P.R_cam_psi = .01;


% dirty derivative parameter
P.tau = 1/5;

% autopilot gains
% roll attitude hold
P.roll_kp      = 1;%1 %****
P.roll_ki      = 0;%.01 %****
P.roll_kd      = .5;%.5 %****

% y-position hold
P.y_kp      = 0.5;%0.9;%0.5;%.495 %****9.659
P.y_ki      = 0.0;%-.01;%-.002; %0%****
P.y_kd      = 0.5;%0.9;%.916 %****

% roll attitude hold
P.pitch_kp      = P.roll_kp;
P.pitch_ki      = P.roll_ki;
P.pitch_kd      = P.roll_kd;

% x-position hold
P.x_kp      = 1;%P.y_kp;%0.5;
P.x_ki      = 0;%P.y_ki;%0;%.1;
P.x_kd      = 0.5;%P.y_kd;%0.5;

% altitude hold
%   P.h_kp      = .01;
%   P.h_ki      = 0;
%   P.h_kd      = 0.1;

P.h_kp      = -1.0;
P.h_ki      = 0;
P.h_kd      = -1.0;

% yaw attitude hold
P.yaw_kp      = 1;%.85 %****10.09
P.yaw_ki      = 0;%.675 %****
P.yaw_kd      = 1;%.85 %****

% camera parameters
P.cam_pix = 480;         % size of (square) pixel array
P.cam_fov = 60*(pi/180); % field of view of camera
P.f = (P.cam_pix/2)/tan(P.cam_fov/2); % focal range
P.eps_s_d = 36;  % desired pixel size in image

% target parameters
P.target_size = 0.1;  % initial location of target
% initial conditions of the target
P.target0 = [...
    0;...  % initial North position
    0;...  % initial East position
    0;...  % initial Down position
    0;...  % initial forward speed
    0;...  % initial heading
    ];
% waypoints for target to follow
P.target_waypoints = [...
    0, 0;...
    3, 0;...
    ];
% commanded speed of target
P.target_speed = .2;

% control gains for feedback control;
P.ctrlGain.k1 = 1;
P.ctrlGain.k2 = 1;

P.accele_bar = 0.5;

