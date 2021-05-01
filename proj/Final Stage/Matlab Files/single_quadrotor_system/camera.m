function [sys,x0,str,ts] = camera(t,x,u,flag,P)
%
% modified 08/10/2014 Liang Sun

switch flag,

  %%%%%%%%%%%%%%%%%%
  % Initialization %
  %%%%%%%%%%%%%%%%%%
  case 0,
    [sys,x0,str,ts] = mdlInitializeSizes(P);

  %%%%%%%%%%
  % Update %
  %%%%%%%%%%
  case 2,                                                
    sys = mdlUpdate(t,x,u,P); 

  %%%%%%%%%%
  % Output %
  %%%%%%%%%%
  case 3,                                                
    sys = mdlOutput(t,x,u,P);

  %%%%%%%%%%%%%
  % Terminate %
  %%%%%%%%%%%%%
  case 9,                                                
    sys = []; % do nothing

  %%%%%%%%%%%%%%%%%%%%
  % Unexpected flags %
  %%%%%%%%%%%%%%%%%%%%
  otherwise
    error(['unhandled flag = ',num2str(flag)]);
end

%end dsfunc

%
%=======================================================================
% mdlInitializeSizes
% Return the sizes, initial conditions, and sample times for the S-function.
%=======================================================================
%
function [sys,x0,str,ts] = mdlInitializeSizes(P)

sizes = simsizes;
sizes.NumContStates  = 0;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 4;
sizes.NumInputs      = 12+7;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;


sys = simsizes(sizes);

x0 = zeros(sizes.NumDiscStates,1);
str = [];
ts  = [P.Tcam 0];  % sample rate of the camera

% end mdlInitializeSizes
%
%=======================================================================
% mdlUpdate
% Handle discrete state updates, sample time hits, and major time step
% requirements.
%==============================2=========================================
%
function sys = mdlUpdate(t,x,u,P)
  sys = [];
%end mdlUpdate

%=======================================================================
% mdlOutput
%==============================2=========================================
%
function sys = mdlOutput(t,x,uu,P)
  NN = 0;
  pn         = uu(1+NN);     % actual states
  pe         = uu(2+NN);
  pd         = uu(3+NN);
  u          = uu(4+NN);
  v          = uu(5+NN);
  w          = uu(6+NN);
  phi        = uu(7+NN);          
  theta      = uu(8+NN);          
  psi        = uu(9+NN);          
  p          = uu(10+NN);
  q          = uu(11+NN);
  r          = uu(12+NN);
  NN = NN+12;
  tn   = uu(NN+1);
  te   = uu(NN+2);
  td   = uu(NN+3);
  tpsi = uu(NN+5);
 
 
  % target in inertial frame
  p_obj_i = [tn; te; td];
  
  % target in the vehicle frame
  p_obj_v = p_obj_i - [pn; pe; pd];
  
  % tranform to the camera frame
  R_v_b = RotMat(phi,theta,psi)';  % vehicle to body
  R_b_g = RotMat(0,-pi/2,0)';      % body to gimbal
  R_g_c = [...                   % gimbal to camera
      0, 1, 0;...
      0, 0, 1;...
      1, 0, 0];
  p_obj_c = (R_g_c * R_b_g * R_v_b) * p_obj_v;
    
  % convert to pixel location
  if p_obj_c(3)<.1,
      qx=-9999;
      qy=-9999;
      target_size = 0;
      target_psi = 0;
  else
    qx =  P.f*(p_obj_c(1)/(p_obj_c(3)));% + P.cam_pix/2;
    qy =  P.f*(p_obj_c(2)/(p_obj_c(3)));% + P.cam_pix/2;
    target_size = P.f*(P.target_size/norm(p_obj_c));
    target_psi = -pi/2 + (tpsi - psi);
  end
  
  % snap output of camera to -9999 if outside field of view
  tmp = 2*P.cam_pix/2;
  %  if qx<0 | qx>P.cam_pix | qy<0 | qy>P.cam_pix,
  if qx<-tmp || qx>tmp || qy<-tmp || qy>tmp,
      qx = -9999;
      qy = -9999;
      target_size = 0;
      target_psi = 0;
  end
 
  sys = [qx; qy; target_size; target_psi];



%%%%%%%%%%%%%%%%%%%%%%%
function R = RotMat(phi,theta,psi)
% Rotation matrix
R = [...
    cos(theta)*cos(psi), ...
    sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi),...
    cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi);...
    cos(theta)*sin(psi),...
    sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi),...
    cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi);...
    -sin(theta),...
    sin(phi)*cos(theta),...
    cos(phi)*cos(theta);...
    ];

