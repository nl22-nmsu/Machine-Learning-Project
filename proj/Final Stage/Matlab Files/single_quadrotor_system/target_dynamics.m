function [sys,x0,str,ts] = target_dynamics(t,x,u,flag,P)
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
    sys=mdlUpdate(t,x,u,P);

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

sizes.NumContStates  = 5;
sizes.NumDiscStates  = 3;
sizes.NumOutputs     = 5+2;
sizes.NumInputs      = 0;
sizes.DirFeedthrough = 0;
sizes.NumSampleTimes = 1;   % at least one sample time is needed

sys = simsizes(sizes);

%
% initialize the initial conditions
%
x0  = [P.target0; 2; 0; 0];

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
  n    = x(1);    % north position (m)
  e    = x(2);    % east position (m)
  d    = x(3);    % position down (negative of altitude) (m)
  V    = x(4);    % linear speed
  psi  = x(5);    % heading (rad)
  wpt  = x(6);    % waypoint counter
  u1   = x(7);    % Vdot
  u2   = x(8);    % psidot
  
  ndot   = V*cos(psi);
  edot   = V*sin(psi);
  ddot   = 0;
  Vdot   = u1; 
  psidot = u2;
  
  xdot = [...
      ndot;...
      edot;...
      ddot;...
      Vdot;...
      psidot;...
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
function sys=mdlUpdate(t,x,u,P)
  n    = x(1);    % north position (m)
  e    = x(2);    % east position (m)
  d    = x(3);    % position down (negative of altitude) (m)
  V    = x(4);    % linear speed
  psi  = x(5);    % heading (rad)
  wpt  = x(6);    % waypoint counter
  u1   = x(7);    % Vdot
  u2   = x(8);    % psidot
  
  % maintain constant speed
  u1 = 1.0*(P.target_speed-V);
  
  % steer vehicle
  ell   = P.target_waypoints(wpt,:) - [n, e];
  psi_d = atan2(ell(2),ell(1));
  u2     = 1.0*wrap(psi_d-psi);
  
  if wpt == 1,
      last_wpt = size(P.target_waypoints,1);
  else
      last_wpt = wpt-1;
  end
  
  if wpt == size(P.target_waypoints,1),
      next_wpt = 1;
  else
      next_wpt = wpt + 1;
  end
  
  a = P.target_waypoints(wpt,:) - P.target_waypoints(last_wpt,:);
  b = [n, e] - P.target_waypoints(wpt,:);
  if a*b' > 0,
      wpt = next_wpt;
  end

% if wpt == 1
    
  

sys = [wpt; u1; u2];

% end mdlUpdate

%
%=============================================================================
% mdlOutputs
% Return the block outputs.
%=============================================================================
%
function sys=mdlOutputs(t,x,u,P)
  n    = x(1);    % north position (m)
  e    = x(2);    % east position (m)
  d    = x(3);    % position down (negative of altitude) (m)
  V    = x(4);    % linear speed
  psi  = x(5);    % heading (rad)
  wpt  = x(6);    % waypoint counter
  u1   = x(7);    % Vdot
  u2   = x(8);    % psidot
  
sys = [n; e; d; V; psi; u1; u2];

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

%
%=============================================================================
% wrap
% wrap angle between +-pi
%=============================================================================
%
function a=wrap(a)
  while a>pi,
      a = a-2*pi;
  end
  while a<-pi,
      a = a+2*pi;
  end

% end wrap

%
%=============================================================================
% wrap2pi
% wrap angle between [0,2*pi]
%=============================================================================
%
function a=wrap2pi(a)
  while a>2*pi,
      a = a-2*pi;
  end
  while a<0,
      a = a+2*pi;
  end

% end wrap2pi
