function y = controller(uu,P)
%
% autopilot for quadrotor
%
% Modification History:
%   08/09/2014 Liang Sun

persistent target_psi_dot_k1 target_psi_ddot
persistent err_old
% initialize persistent variables at beginning of simulation
t = uu(1);
NN = 1;
camera.qx   = uu(1+NN);
camera.qy   = uu(2+NN);
camera.target_size = uu(3+NN);
camera.target_psi  = uu(4+NN);

NN = 1+4;
quadrotor.pn     = uu(1+NN);    % north position (m)
quadrotor.pe     = uu(2+NN);    % east position (m)
quadrotor.pd     = uu(3+NN);    % position down (negative of altitude) (m)
quadrotor.u      = uu(4+NN);    % velocity along body x-axis (m/s)
quadrotor.v      = uu(5+NN);    % velocity along body y-axis (m/s)
quadrotor.w      = uu(6+NN);    % velocity along body z-axis (m/s)
quadrotor.phi    = uu(7+NN);    % roll angle (rad)
quadrotor.theta  = uu(8+NN);    % pitch angle  (rad)
quadrotor.psi    = uu(9+NN);    % yaw angle (rad)
quadrotor.p      = uu(10+NN);   % roll rate  (rad/s)
quadrotor.q      = uu(11+NN);   % pitch rate (rad/s)
quadrotor.r      = uu(12+NN);   % yaw rate (rad/s)

NN = 1+4+12;
target.n      = uu(1+NN);    % north position (m)
target.e      = uu(2+NN);    % east position (m)
target.d      = uu(3+NN);    % position down (negative of altitude) (m)
target.V      = uu(4+NN);    % linear speed
target.psi    = uu(5+NN);    % heading (rad)
target.Vdot   = uu(6+NN);    % Vdot
target.psidot = uu(7+NN);    % psidot

% differentiate psidot
if t<P.Ts
    target_psi_ddot=0;
    target_psi_dot_k1 = target.psidot;
else
    
    target_psi_ddot = (2*P.tau-P.Ts)/(2*P.tau+P.Ts)*target_psi_ddot...
        + (2/(2*P.tau+P.Ts))*(target.psidot - target_psi_dot_k1);
    target_psi_dot_k1 = target.psidot;
end
% target.psi_ddot = target_psi_ddot;
target.psi_ddot = 0;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% implement autopilot modes

%---------------------------------------------------------------
% compute feed forward terms
% compute accelerations in the body frame
% mx_ddot = target.Vdot*cos(quadrotor.psi)-target.V*sin(quadrotor.psi)*target.psidot;
% my_ddot = target.Vdot*sin(quadrotor.psi)+target.V*cos(quadrotor.psi)*target.psidot;
mx_ddot = 0;
my_ddot = 0;

% desired acceleration

timeToStop = 3;

% if t<=100
%     p_d = [0;0;-2];    
% else
%     p_d = [2;0;-2];
% end

p_d = [P.orbit.radius * sin(P.quadrotor.V/P.orbit.radius * t);...
    P.orbit.radius * cos(P.quadrotor.V/P.orbit.radius * t);...
    -P.height_d] ;

% p_d = [2;0;-2];
psi_d = atan2(p_d(1),p_d(2))+pi/2;

% p_d = [2;2;-2]; 
p_d_dot  = [0;0;0];
p_d_ddot = [0;0;0];
err = [quadrotor.pn;quadrotor.pe;quadrotor.pd] - p_d;
if t<P.Ts
    err_dot = [0;0;0];
    err_old = err;
else
    err_dot = (err-err_old)/P.Ts;
    err_old = err;
end
u_d = p_d_ddot - (P.ctrlGain.k1+P.ctrlGain.k2)*err_dot ...
               - (1+P.ctrlGain.k1*P.ctrlGain.k2)*err;
u_x = u_d(1);
u_y = u_d(2);
u_z = u_d(3);

% u_overall = u_d + u_c;

%---------------------------------------------------------------
% regulate altitude based on size of object in image
%altitude_d = 1.25;
h = -quadrotor.pd;
error_h = -h+P.height_d;
% u_z = altitude_hold(error_h, t, P);
F_thrust = P.m*(P.g - u_z)/cos(quadrotor.phi)/cos(quadrotor.theta);

%---------------------------------------------------------------
% regulate pitch angle to drive x-axis error to zero


% error_x = 0;
% u_x  = -mx_ddot+longitudinal_position_hold(error_x, 0, t, P);
% u_x = sat(u_x,-P.accele_bar,P.accele_bar);
decouple_x = u_x/(P.g - u_z);
theta_c  = -atan(decouple_x);

Tq = pitch_attitude_hold(quadrotor.theta, theta_c, quadrotor.q, t, P);

%---------------------------------------------------------------
% regulate roll angle to drive y-axis error to zero
% error_y = 0;
% u_y  = my_ddot + lateral_position_hold(error_y, 0, t, P);
decouple_y = u_y*cos(quadrotor.theta)/(P.g - u_z);

phi_c = atan(decouple_y);
Tp = roll_attitude_hold(quadrotor.phi, phi_c, quadrotor.p, t, P);

%---------------------------------------------------------------
% regulate heading to align with orientation of object in image

err_psi = quadrotor.psi;
Tr = P.Jz*target.psi_ddot + heading_hold(err_psi, quadrotor.r, t, P);
if Tr>1
    a =1;
end
% control output
pwm = [...
    1/4/P.kmotor_thrust, 0, 1/2/P.l/P.kmotor_thrust,  -1/4/P.kmotor_torque;...
    1/4/P.kmotor_thrust, -1/2/P.l/P.kmotor_thrust, 0,  1/4/P.kmotor_torque;...
    1/4/P.kmotor_thrust, 0, -1/2/P.l/P.kmotor_thrust, -1/4/P.kmotor_torque;...
    1/4/P.kmotor_thrust, 1/2/P.l/P.kmotor_thrust, 0,   1/4/P.kmotor_torque;...
    ]*[F_thrust; Tp; Tq; Tr];
% pwm = [...
%     1/4/P.kmotor_thrust, 0, 1/2/P.l/P.kmotor_thrust,  1/4/P.kmotor_torque;...
%     1/4/P.kmotor_thrust, -1/2/P.l/P.kmotor_thrust, 0, -1/4/P.kmotor_torque;...
%     1/4/P.kmotor_thrust, 0, -1/2/P.l/P.kmotor_thrust, 1/4/P.kmotor_torque;...
%     1/4/P.kmotor_thrust, 1/2/P.l/P.kmotor_thrust, 0,  -1/4/P.kmotor_torque;...
%     ]*[F_thrust; Tp; Tq; Tr];

% % desired states (for visulation and debugging)
% xd = [...
%     0;...                % xerror: position error along body x-axis
%     0;...                % yerror: position error along body y-axis
%     -1;...                % zerror:  position error along body z-axis (in pixels)
%     0;...                % u: desired velocity along body x-axis
%     0;...                % v: desired velocity along body v-axis
%     0;...                % w: desired velocity along body z-axis
%     0;%phi_c;...            % phi:   desired roll angle
%     0;%theta_c;...          % theta: desired pitch angle
%     0;...                % psi:   desired heading
%     0;...                % desired roll rate
%     0;...                % desired pitch rate
%     0;...                % desired yaw rate
%     ];


y = [pwm];



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Autopilot functions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% roll_attitude_hold
%  - regulate roll_attitude
%  - produces desired roll rate
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function torque = roll_attitude_hold(phi, phi_c, p, t, P);
persistent uik1;
persistent udk1;
persistent errork1;
persistent phik1;
% initialize persistent variables at beginning of simulation
if t<P.Ts,
    uik1    = 0;
    errork1 = 0;
end

% error equation
error = phi_c - phi;

% proportional term
up = P.roll_kp * error;

% integral term
ui = uik1 + P.roll_ki * P.Ts/2 * (error + errork1);

% derivative term
ud = -P.roll_kd*p;

% implement PID control
torque = up + ui + ud;

% update stored variables
uik1    = ui;
errork1 = error;



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% lateral_position_hold
%  - regulate lateral position
%  - produces desired roll command
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function u_y  = lateral_position_hold(error, pydot, t, P)
persistent uik1;
persistent errork1;
persistent vhatk1;
% initialize persistent variables at beginning of simulation
if t<P.Ts,
    uik1    = 0;
    errork1 = 0;
    vhatk1 = 0;
end

% proportional term
up = -P.y_kp * error;

% integral term
ui = uik1 + P.y_ki * P.Ts/2 * (error + errork1);

% dirty derivative of pe to get vhat
vhat = (2*P.tau-P.Ts)/(2*P.tau+P.Ts)*vhatk1 + (2/(2*P.tau+P.Ts))*(error - errork1);
ud = -P.y_kd*vhat;
%  ud = -P.y_kd*pydot;


% implement PID control
u_y = up + ui + ud;

% saturate the roll command
%u_y = sat(u_y, 20*pi/180, -20*pi/180);

% update stored variables
uik1    = ui;
errork1 = error;
vhatk1  = vhat;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% pitch_attitude_hold
%  - regulate pitch attitude
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function torque = pitch_attitude_hold(theta, theta_c, q, t, P)
persistent uik1;
persistent udk1;
persistent errork1;
persistent thetak1;
% initialize persistent variables at beginning of simulation
if t<P.Ts,
    uik1    = 0;
    errork1 = 0;
end

% error equation
error = theta_c - theta;

% proportional term
up = P.pitch_kp * error;

% integral term
ui = uik1 + P.pitch_ki * P.Ts/2 * (error + errork1);

% derivative term
ud = -P.pitch_kd*q;


% implement PID control
torque = up + ui + ud;

% update stored variables
uik1    = ui;
errork1 = error;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% longitudinal_position_hold
%  - regulate longitudinal position
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function u_x  = longitudinal_position_hold(error, pxdot, t, P)
persistent ui;
persistent errork1;
persistent vhatk1;
% initialize persistent variables at beginning of simulation
if t<P.Ts,
    ui    = 0;
    errork1 = 0;
    vhatk1 = 0;
end

% proportional term1
up = P.x_kp * (error);

% integral term
ui = ui + P.x_ki * P.Ts/2 * (error + errork1);

% dirty derivative of pe to get vhat
vhat = (2*P.tau-P.Ts)/(2*P.tau+P.Ts)*vhatk1 + (2/(2*P.tau+P.Ts))*(error - errork1);
ud = P.x_kd*vhat;
%  ud = P.x_kd*pxdot;


% implement PID control
u_x = up + ui + ud;

% saturate the roll command
%u_x = sat(u_x, 20*pi/180, -20*pi/180);

% update stored variables
errork1 = error;
vhatk1 = vhat;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% altitude hold
%  - regulate altitude
%  - produces force command
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function Force = altitude_hold(error, t, P)
persistent uik1;
persistent error_k1;
persistent error_dot_k1;
% initialize persistent variables at beginning of simulation
if t<P.Ts,
    uik1       = 0;
    error_k1 = 0;
    error_dot_k1 = 0;
end

% proportional term
up = P.h_kp * error;

% integral term
ui = uik1 + P.h_ki * P.Ts/2 * (error + error_k1);

% dirty derivative of error
error_dot = (2*P.tau-P.Ts)/(2*P.tau+P.Ts)*error_dot_k1 ...
    + (2/(2*P.tau+P.Ts))*(error - error_k1);
ud = P.h_kd * error_dot;

% implement PID control
Force = up + ui + ud;



% saturate the force command
%F = sat(F, 20*pi/180, -20*pi/180);

% update stored variables
uik1       = ui;
error_k1 = error;
error_dot_k1 = error_dot;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% heading_hold
%  - regulate heading
%  - produces torque about the yaw axis
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function torque = heading_hold(error, r, t, P)
persistent uik1;
persistent errork1;
% initialize persistent variables at beginning of simulation
if t<P.Ts,
    uik1    = 0;
    errork1 = 0;
end

%  error = + pi/2 + eps_psi ;

% proportional term
up = -P.yaw_kp * error;

% integral term
ui = uik1 + P.yaw_ki * P.Ts/2 * (error + errork1);

% derivative term
ud = -P.yaw_kd*r;


% implement PID control
torque = up + ui + ud;

% update stored variables
uik1    = ui;
errork1 = error;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% sat
%   - saturation function
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function out = sat(in, up_limit, low_limit)
if in > up_limit,
    out = up_limit;
elseif in < low_limit;
    out = low_limit;
else
    out = in;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% initialize the persistent variables
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [xhat, sensors, target] = initialize_persistent_variables(P)
xhat.px    = 0;
xhat.py    = 0;
xhat.pz    = -P.h;
xhat.pxdot     = 0;
xhat.pydot     = 0;
xhat.pzdot     = 0;
xhat.phi   = 0;
xhat.theta = 0;
xhat.psi   = 0;

for i=1:P.cam_delay,
    sensors(i).eps_x     = 0;
    sensors(i).eps_y     = 0;
    sensors(i).eps_s     = 0;
    sensors(i).eps_psi   = 0;
    sensors(i).new_camera_data_flag = 0;
    sensors(i).gyro_x    = 0;
    sensors(i).gyro_y    = 0;
    sensors(i).gyro_z    = 0;
    sensors(i).accel_x   = 0;
    sensors(i).accel_y   = 0;
    sensors(i).accel_z   = 0;
    sensors(i).altimeter = 0;
    sensors(i).p         = 0;
    sensors(i).q         = 0;
    sensors(i).r         = 0;
    target(i).speed      = 0;
    target(i).vdot       = 0;
    target(i).psi_dot    = 0;
    target(i).psi_ddot   = 0;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% process sensors
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [sensors, t] = process_sensors(sensors,uu,P)

% shift old sensor value
for i=P.cam_delay:-1:2,
    sensors(i).eps_x     = sensors(i-1).eps_x;
    sensors(i).eps_y     = sensors(i-1).eps_y;
    sensors(i).eps_s     = sensors(i-1).eps_s;
    sensors(i).eps_psi   = sensors(i-1).eps_psi;
    sensors(i).new_camera_data_flag = sensors(i-1).new_camera_data_flag;
    sensors(i).gyro_x    = sensors(i-1).gyro_x;
    sensors(i).gyro_y    = sensors(i-1).gyro_y;
    sensors(i).gyro_z    = sensors(i-1).gyro_z;
    sensors(i).accel_x   = sensors(i-1).accel_x;
    sensors(i).accel_y   = sensors(i-1).accel_y;
    sensors(i).accel_z   = sensors(i-1).accel_z;
    sensors(i).altimeter = sensors(i-1).altimeter;
    sensors(i).p         = sensors(i-1).p;
    sensors(i).q         = sensors(i-1).q;
    sensors(i).r         = sensors(i-1).r;
end

% camera measurements
NN = 1;
sensors(1).eps_x   = uu(1+NN);            % pixel location along camera x-axis
sensors(1).eps_y   = uu(2+NN);            % pixel location along camera y-axis
sensors(1).eps_s   = uu(3+NN);            % sizer of target in pixels
sensors(1).eps_psi = uu(4+NN);            % orientation of target in image

% set flag when new camera measurement is received
sensors(1).new_camera_data_flag = 0;
if P.cam_delay>1,
    if (...
            sensors(2).eps_x~=sensors(1).eps_x || ...
            sensors(2).eps_y~=sensors(1).eps_y || ...
            sensors(2).eps_s~=sensors(1).eps_s || ...
            sensors(2).eps_psi~=sensors(1).eps_psi ),
        sensors(1).new_camera_data_flag = 1;
    end
end

% gyro measurements
NN = NN+4;
sensors(1).gyro_x  = uu(1+NN);
sensors(1).gyro_y  = uu(2+NN);
sensors(1).gyro_z  = uu(3+NN);

% remove biases from recent gyro measurements
% remove (95%) of bias from sensors
M = 0.95;
M = 1.0;
sensors(1).gyro_x = sensors(1).gyro_x - M*P.gyro_x_bias;
sensors(1).gyro_y = sensors(1).gyro_y - M*P.gyro_y_bias;
sensors(1).gyro_z = sensors(1).gyro_z - M*P.gyro_z_bias;


% accelerometer measurements
NN = NN + 3;
sensors(1).accel_x = uu(1+NN);
sensors(1).accel_y = uu(2+NN);
sensors(1).accel_z = uu(3+NN);

% remove biases from recent accelerometer measurements
% remove (95%) of bias from sensors
M = 0.95;
M = 1.0;
sensors(1).accel_x = sensors(1).accel_x - M*P.accel_x_bias;
sensors(1).accel_y = sensors(1).accel_y - M*P.accel_y_bias;
sensors(1).accel_z = sensors(1).accel_z - M*P.accel_z_bias;


% ultrasonic altimeter measurement
NN = NN + 3;
sensors(1).altimeter = uu(1+NN);

% low pass filter gyros
%     alpha = exp(-P.lpf_gyro*P.Ts);
%     sensors(1).p = alpha * sensors(1).p + (1-alpha) * sensors(1).gyro_x;
%     sensors(1).q = alpha * sensors(1).q + (1-alpha) * sensors(1).gyro_y;
%     sensors(1).r = alpha * sensors(1).r + (1-alpha) * sensors(1).gyro_z;
sensors(1).p = sensors(1).gyro_x;
sensors(1).q = sensors(1).gyro_y;
sensors(1).r = sensors(1).gyro_z;

% time
NN = NN+1;
t     = uu(1+NN);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% use real states
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [xhat, sensors] = use_real_states(xhat,sensors,uu,t,P)
NN = 12+3;
%   pn = uu(1+NN);
%   pe = uu(2+NN);
xhat.pz = uu(3+NN);
xhat.pxdot  = uu(4+NN);
xhat.pydot  = uu(5+NN);
xhat.pzdot  = uu(6+NN);
xhat.phi = uu(7+NN);
xhat.theta = uu(8+NN);
xhat.psi   = uu(9+NN);
sensors(1).p  = uu(10+NN);
sensors(1).q  = uu(11+NN);
sensors(1).r  = uu(12+NN);

% estimate px, py, psi from camera
xhat.px = xhat.pz*tan(xhat.theta - sensors(1).eps_y * (P.cam_fov/P.cam_pix));
xhat.py = -xhat.pz*tan(xhat.phi - sensors(1).eps_x * (P.cam_fov/P.cam_pix));
xhat.psi = pi/2 + sensors(1).eps_psi;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% get communication packet from target
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function  target = get_comm_from_target(target,uu,t,P)
persistent target_psi_ddot
persistent target_psi_dot_k1
% initialize persistent variables at beginning of simulation
if t<P.Ts,
    target_psi_ddot = 0;
    target_psi_dot_k1 = 0;
end

% shift old target value
for i=P.cam_delay:-1:2,
    target(i).speed     = target(i-1).speed;
    target(i).vdot     = target(i-1).vdot;
    target(i).psi_dot     = target(i-1).psi_dot;
    target(i).psi_ddot     = target(i-1).psi_ddot;
end

% read data from comm packet
NN = 1;  % time
NN = NN + 4; % camera
NN = NN + 7; % sensors
target(1).speed  = uu(1+NN);
target(1).vdot   = uu(2+NN);
target(1).psi_dot = uu(3+NN);

% differentiate psidot
target_psi_ddot = (2*P.tau-P.Ts)/(2*P.tau+P.Ts)*target_psi_ddot...
    + (2/(2*P.tau+P.Ts))*(target(1).psi_dot - target_psi_dot_k1);
target_psi_dot_k1 = target(1).psi_dot;
target(1).psi_ddot = target_psi_ddot;
