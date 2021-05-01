function [sys,x0,str,ts] = quadrotor_dynamics(t,x,u,flag,P)
%
switch flag,

  %%%%%%%%%%%%%%%%%%
  % Initialization %
  %%%%%%%%%%%%%%%%%%
  case 0,
    [sys,x0,str,ts]=mdlInitializeSizes(P);

  %%%%%%%%%%%%%%%
  % Derivatives %
  %%%%%%%%%%%%%%%
  case 1,
    sys=mdlDerivatives(t,x,u,P);

  %%%%%%%%%%
  % Update %
  %%%%%%%%%%
  case 2,
    sys=mdlUpdate(t,x,u);

  %%%%%%%%%%%
  % Outputs %
  %%%%%%%%%%%
  case 3,
    sys=mdlOutputs(t,x,u,P);

  %%%%%%%%%%%%%%%%%%%%%%%
  % GetTimeOfNextVarHit %
  %%%%%%%%%%%%%%%%%%%%%%%
  case 4,
    sys=mdlGetTimeOfNextVarHit(t,x,u);

  %%%%%%%%%%%%%
  % Terminate %
  %%%%%%%%%%%%%
  case 9,
    sys=mdlTerminate(t,x,u);

  %%%%%%%%%%%%%%%%%%%%
  % Unexpected flags %
  %%%%%%%%%%%%%%%%%%%%
  otherwise
    error(['Unhandled flag = ',num2str(flag)]);

end

% end sfuntmpl

%
%=============================================================================
% mdlInitializeSizes
% Return the sizes, initial conditions, and sample times for the S-function.
%=============================================================================
%
function [sys,x0,str,ts]=mdlInitializeSizes(P)
%
% call simsizes for a sizes structure, fill it in and convert it to a
% sizes array.
%
% Note that in this example, the values are hard coded.  This is not a
% recommended practice as the characteristics of the block are typically
% defined by the S-function parameters.
%
sizes = simsizes;

sizes.NumContStates  = 12;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 12;
sizes.NumInputs      = 6;
sizes.DirFeedthrough = 0;
sizes.NumSampleTimes = 1;   % at least one sample time is needed

sys = simsizes(sizes);

%
% initialize the initial conditions
%
x0  = P.x0;

%
% str is always an empty matrix
%
str = [];

%
% initialize the array of sample times
%
ts  = [0 0];

% end mdlInitializeSizes

%
%=============================================================================
% mdlDerivatives
% Return the derivatives for the continuous states.
%=============================================================================
%
function sys=mdlDerivatives(t,x,uu,P)
  % process states
  pn = x(1);    % north position (m)
  pe = x(2);    % east position (m)
  pd = x(3);    % position down (negative of altitude) (m)
  u  = x(4);    % velocity along body x-axis (m/s)
  v  = x(5);    % velocity along body y-axis (m/s)
  w  = x(6);    % velocity along body z-axis (m/s)
  phi = x(7);   % roll angle (rad)
  theta = x(8); % pitch angle  (rad)
  psi = x(9);   % yaw angle (rad)
  p = x(10);    % roll rate  (rad/s)
  q = x(11);    % pitch rate (rad/s)
  r = x(12);    % yaw rate (rad/s)
  

  % process inputs
   pwm_f = uu(1); %sat(uu(1),0,1);  % pwm commend for front motor
   pwm_r = uu(2); %sat(uu(2),0,1);  % pwm commend for right motor
   pwm_b = uu(3); %sat(uu(3),0,1);  % pwm commend for back motor
   pwm_l = uu(4); %sat(uu(4),0,1);  % pwm commend for left motor
   wn    = uu(5); % wind along the North coordinate
   we    = uu(6); % wind along the East coordinate


  % computer thrust produced by each motor
   thrust_front = P.kmotor_thrust*pwm_f;
   thrust_right = P.kmotor_thrust*pwm_r;
   thrust_back  = P.kmotor_thrust*pwm_b;
   thrust_left  = P.kmotor_thrust*pwm_l;
 
  % computer torque produced by each motor
   torque_front = P.kmotor_torque*pwm_f;
   torque_right = P.kmotor_torque*pwm_r;
   torque_back  = P.kmotor_torque*pwm_b;
   torque_left  = P.kmotor_torque*pwm_l;

   % compute body frame torques
  T_p = P.l*(thrust_left - thrust_right); % torque about roll axis
  T_q = P.l*(thrust_front - thrust_back); % torque about pitch axis
%   T_r = torque_front - torque_right + torque_back - torque_left;  % torque about yaw axis
  T_r = -torque_front + torque_right - torque_back + torque_left;  % torque about yaw axis

  % compute body frame force
  F_thrust = [0; 0; -(thrust_front + thrust_right + thrust_back + thrust_left)];
  
  % force due to gravity in the body frame
  F_gravity = P.m*P.g*[-sin(theta); cos(theta)*sin(phi); cos(theta)*cos(phi)];
  
  % vec_cable = [0;0;0] - [pn;pe;pd];
  % unit_vec_cable = vec_cable/norm(vec_cable);
  % F_cable_inertial = P.F_mag * unit_vec_cable;
  % F_cable_body = R_v_b * F_cable_inertial;  % figure out R_i_b

  % compute system dynamics R is b to v.
  R        = [ cos(psi), -sin(psi), 0;...
               sin(psi), cos(psi), 0;...
               0, 0, 1]...
            *[ cos(theta), 0, sin(theta);...
               0, 1, 0;...
               -sin(theta), 0, cos(theta)]...
            *[ 1, 0, 0;...
               0, cos(phi), -sin(phi);...
               0, sin(phi), cos(phi)];
  pndot    = R(1,1)*u + R(1,2)*v + R(1,3)*w + wn; % Application of north wind
  pedot    = R(2,1)*u + R(2,2)*v + R(2,3)*w + we; %Application of east wind 
  pddot    = R(3,1)*u + R(3,2)*v + R(3,3)*w;
  udot     = r*v-q*w + 1/P.m*( F_gravity(1) + F_thrust(1) ); 
  vdot     = p*w-r*u + 1/P.m*( F_gravity(2) + F_thrust(2) ); 
  wdot     = q*u-p*v + 1/P.m*( F_gravity(3) + F_thrust(3) );
  phidot   = p + sin(phi)*tan(theta)*q + cos(phi)*tan(theta)*r;
  thetadot = cos(phi)*q - sin(phi)*r;
  psidot   = sin(phi)/cos(theta)*q + cos(phi)/cos(theta)*r;
  pdot     = (P.Jy-P.Jz)/P.Jx*q*r + (1/P.Jx) * T_p;
  qdot     = (P.Jz-P.Jx)/P.Jy*p*r + (1/P.Jy) * T_q;
  rdot     = (P.Jx-P.Jy)/P.Jz*p*q + (1/P.Jz) * T_r;

%   udot = sat(udot,-P.accele_bar,P.accele_bar);
%   vdot = sat(vdot,-P.accele_bar,P.accele_bar);
%   wdot = sat(wdot,-P.accele_bar,P.accele_bar);
  % compute system dynamics
  
  xdot = [...
      pndot; pedot; pddot;...
      udot; vdot; wdot;...
      phidot; thetadot; psidot;...
      pdot; qdot; rdot...
      ];
  
sys = xdot;

% end mdlDerivatives

%
%=============================================================================
% mdlUpdate
% Handle discrete state updates, sample time hits, and major time step
% requirements.
%=============================================================================
%
function sys=mdlUpdate(t,x,u)

sys = [];

% end mdlUpdate

%
%=============================================================================
% mdlOutputs
% Return the block outputs.
%=============================================================================
%
function sys=mdlOutputs(t,x,u,P)
  
sys = x;

% end mdlOutputs

%
%=============================================================================
% mdlGetTimeOfNextVarHit
% Return the time of the next hit for this block.  Note that the result is
% absolute time.  Note that this function is only used when you specify a
% variable discrete-time sample time [-2 0] in the sample time array in
% mdlInitializeSizes.
%=============================================================================
%
function sys=mdlGetTimeOfNextVarHit(t,x,u)

sampleTime = 1;    %  Example, set the next hit to be one second later.
sys = t + sampleTime;

% end mdlGetTimeOfNextVarHit

%
%=============================================================================
% mdlTerminate
% Perform any end of simulation tasks.
%=============================================================================
%
function sys=mdlTerminate(t,x,u)

sys = [];

% end mdlTerminate

%
%=============================================================================
% sat
% saturates the input between high and low
%=============================================================================
%
function out=sat(in, low, high)

  if in < low,
      out = low;
  elseif in > high,
      out = high;
  else
      out = in;
  end

% end sat
