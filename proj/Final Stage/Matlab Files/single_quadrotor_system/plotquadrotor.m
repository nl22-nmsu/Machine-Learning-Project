function [sys,x0,str,ts] = plotquadrotor(t,x,u,flag,P)
%
% Modified:
% 08/10/2014 Liang Sun

switch flag,

  %%%%%%%%%%%%%%%%%%
  % Initialization %
  %%%%%%%%%%%%%%%%%%
  case 0,
    [sys,x0,str,ts] = mdlInitializeSizes;

  %%%%%%%%%%
  % Update %
  %%%%%%%%%%
  case 2,                                                
    sys = mdlUpdate(t,x,u,P); 

  %%%%%%%%%%
  % Output %
  %%%%%%%%%%
  case 3,                                                
    sys = [];

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
function [sys,x0,str,ts] = mdlInitializeSizes

sizes = simsizes;
sizes.NumContStates  = 0;
sizes.NumDiscStates  = 4;
sizes.NumOutputs     = 0;
sizes.NumInputs      = 12+4+7;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;


sys = simsizes(sizes);

x0 = zeros(sizes.NumDiscStates,1);
str = [];
ts  = [.1 0]; 

% end mdlInitializeSizes
%
%=======================================================================
% mdlUpdate
% Handle discrete state updates, sample time hits, and major time step
% requirements.
%=======================================================================
%
function xup = mdlUpdate(t,x,uu,P)

  initialize    = x(1);       % initial graphics
  fig_quadrotor = x(2);       % quadrotor figure handle
  fig_target    = x(3);       % target figure handle
  fig_cam_obj   = x(4);       % position figure handle
  %fig_cable    = x(5);
  NN = 0;
  pn            = uu(1+NN);       % inertial North position     
  pe            = uu(2+NN);       % inertial East position
  pd            = uu(3+NN);           
  u             = uu(4+NN);       
  v             = uu(5+NN);       
  w             = uu(6+NN);       
  phi           = uu(7+NN);       % roll angle     
  theta         = uu(8+NN);       % pitch angle     
  psi           = uu(9+NN);       % yaw angle     
  p             = uu(10+NN);       % roll rate
  q             = uu(11+NN);       % pitch rate     
  r             = uu(12+NN);       % yaw rate    
  NN = NN + 12;
  qx            = uu(1+NN);       % pixel x location
  qy            = uu(2+NN);       % pixel y location
  qsize         = uu(3+NN);       % pixel size
  qpsi          = uu(4+NN);       % target orientation in image
  NN = NN + 4;
  tn            = uu(1+NN);       % target x location
  te            = uu(2+NN);       % target y location
  td            = uu(3+NN);       % target z location
  tpsi          = uu(5+NN);       % target orientation in inertial frame
  

  if initialize==0, 
    % initialize the plot
    initialize = 1;
    figure(1), clf
    
    % plot quadrotor
    % subplot(221)
%     subplot('position',[0.01, 0.01, 0.6, 0.6]);
%    axis(1.0*[-1, 1, -1, 1, -.1, 1.9]) % units are centimeters
%     axis([-1,5,-3,5,-.1, 2]);
%     set(gca,'visible','off')
    fig_quadrotor = quadrotorPlot(pn, pe, pd, phi, theta, psi, P, [], 'normal');
    % fig_cable = drawCable(pn, pe, pd);
    xlabel('East (m)')
    ylabel('North (m)')
    zlabel('Altitude (m)')
    title('3D View')
    hold on
    % plot waypoints
%     plot3(P.target_waypoints(:,2), P.target_waypoints(:,1), zeros(size(P.target_waypoints(:,1))));
%     fig_target = targetPlot(tn,te,td,tpsi,P.target_size,[],'r','normal');
%     title('Attitude (roll, pitch, yaw)')
    view(-45,45)
    xmax = 4;
    xmin = -4;
    ymax = 4;
    ymin = -4;
    zmax = 2;
    zmin = 0;
    axis([xmin,xmax,ymin,ymax,zmin,zmax]);
%    view(0,90)  % top down view to check heading hold loop
%    view(0,0)   % side view to check altitude hold loop
    grid on

    % plot camera view
%     figure(2),clf
%     handle=subplot('position',[0.61, 0.61, 0.3, 0.3]);
%     tmp = P.cam_pix/2;
%     set(handle,'XAxisLocation','top','XLim',[-tmp,tmp],'YLim',[-tmp,tmp]);
%     axis ij
%     hold on
%     fig_cam_obj = targetCamPlot(qx,qy,0,qpsi,qsize,[],'r','normal');
%     xlabel('px (pixels)')
%     ylabel('py (pixels)')
%     title('Camera View')
    
    
  else  % do this at every time step

    % plot quadrotor
     quadrotorPlot(pn, pe, pd, phi, theta, psi, P, fig_quadrotor);
     % drawCable(pn,pe,pd,....)
%      targetPlot(tn,te,td,tpsi,P.target_size,fig_target);     
    % plot camera view
%      targetCamPlot(qx,qy,0,qpsi,qsize,fig_cam_obj);

end
  
%   xup = [initialize; fig_quadrotor; fig_target; fig_cam_obj];
xup(1) = initialize;
xup(2) = fig_quadrotor;
xup(3) = fig_target;
xup(4) = fig_cam_obj;


%end mdlUpdate

%----------------------------------------------------------------------
%----------------------------------------------------------------------
% User defined functions
%----------------------------------------------------------------------
%----------------------------------------------------------------------

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function handle = quadrotorPlot(pn, pe, pd, phi, theta, psi, P, handle, mode);
  % plot of quadrotor in contraption - rolling motion

  psi = -psi;
  
  %----quadrotor vertices------
  drawParam = defineParameters;
  [Vert_quad, Face_quad, colors_quad]       = quadrotorVertFace(drawParam);
  % rotate vertices by phi, theta, psi
  Vert_quad = rotateVert(Vert_quad, phi, theta, psi);
  % transform vertices from NED to XYZ
  R = [...
      0, 1, 0;...
      1, 0, 0;...
      0, 0, 1;...
      ];
  Vert_quad = Vert_quad*R;
  % translate vertices in XYZ
  Vert_quad = translateVert(Vert_quad, [pe; pn; -pd]);

  %----field-of-view vertices------
  [Vert_fov, Face_fov, colors_fov]          = fovVertFace(pd,phi,theta,psi,drawParam,P);
  Vert_fov = translateVert(Vert_fov, [pe; pn; -pd]);

  % collect all vertices and faces
  V = [...
      Vert_quad;...
      Vert_fov;...
      ];
  F = [...
      Face_quad;...
      size(Vert_quad,1) + Face_fov;...
      ]; 
  patchcolors = [...
      colors_quad;...
      colors_fov;...
      ];


  if isempty(handle),
  handle = patch('Vertices', V, 'Faces', F,...
                 'FaceVertexCData',patchcolors,...
                 'FaceColor','flat',...
                 'EraseMode', mode);
  else
    set(handle,'Vertices',V,'Faces',F);
    drawnow
  end
 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% define the quadrotor parameters
function drawParam = defineParameters

% parameters
drawParam.r = 0.1;   % radius of the rotor
drawParam.l = 0.1;   % length of connecting rods
drawParam.lw = 0.01; % width of rod
drawParam.w = 0.1;   % width of the center pod
drawParam.w_rail = 0.01; % width of the rail
drawParam.l_rail = 5; % length of the rail
drawParam.N = 10;     % number of points defining rotor

% define colors for faces
drawParam.myred = [1, 0, 0];
drawParam.mygreen = [0, 1, 0];
drawParam.myblue = [0, 0, 1];
drawParam.myyellow = [1,1,0];

%%%%%%%%%%%%%%%%%%%%%%%
function Vert=rotateVert(Vert,phi,theta,psi);
  % define rotation matrix
  R_roll = [...
          1, 0, 0;...
          0, cos(phi), -sin(phi);...
          0, sin(phi), cos(phi)];
  R_pitch = [...
          cos(theta), 0, sin(theta);...
          0, 1, 0;...
          -sin(theta), 0, cos(theta)];
  R_yaw = [...
          cos(psi), -sin(psi), 0;...
          sin(psi), cos(psi), 0;...
          0, 0, 1];
  R = R_roll'*R_pitch'*R_yaw';

  % rotate vertices
  Vert = (R*Vert')';
  
% end rotateVert

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% translate vertices by column vector T
function Vert = translateVert(Vert, T)

  Vert = Vert + repmat(T', size(Vert,1),1);

% end translateVert

%%%%%%%%%%%%%%%%%%%%%%%
function [X,Y,Z]=rotateXYZ(X,Y,Z,phi,theta,psi);
  % define rotation matrix
  R_roll = [...
          1, 0, 0;...
          0, cos(phi), -sin(phi);...
          0, sin(phi), cos(phi)];
  R_pitch = [...
          cos(theta), 0, sin(theta);...
          0, 1, 0;...
          -sin(theta), 0, cos(theta)];
  R_yaw = [...
          cos(psi), -sin(psi), 0;...
          sin(psi), cos(psi), 0;...
          0, 0, 1];
  R = R_roll'*R_pitch'*R_yaw';

  % rotate vertices
  pts = [X, Y, Z]*R;
  X = pts(:,1);
  Y = pts(:,2);
  Z = pts(:,3);
  
% end rotateVert

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% translate vertices by column vector T
function [X,Y,Z] = translateXYZ(X,Y,Z,T)

  X = X + T(1)*ones(size(X));
  Y = Y + T(2)*ones(size(Y));
  Z = Z + T(3)*ones(size(Z));

% end translateXYZ

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [Vert_quad, Face_quad, colors_quad] = quadrotorVertFace(drawParam)
% Vertices = [x,y,z] position of each vertex
% Faces = defines how the vertices are connnected for form faces.  Each set
% of our vertices defines one face.


%--------- vertices and faces for center pod ------------
% vertices of the center pod
Vert_center = [...
    drawParam.w/2, drawParam.w/2, drawParam.w/2;...
    drawParam.w/2, drawParam.w/2, -drawParam.w/2;...
    drawParam.w/2, -drawParam.w/2, -drawParam.w/2;...
    drawParam.w/2, -drawParam.w/2, drawParam.w/2;...
    -drawParam.w/2, drawParam.w/2, drawParam.w/2;...
    -drawParam.w/2, drawParam.w/2, -drawParam.w/2;...
    -drawParam.w/2, -drawParam.w/2, -drawParam.w/2;...
    -drawParam.w/2, -drawParam.w/2, drawParam.w/2;...
    ];
% define faces of center pod
Face_center = [...
        1, 2, 3, 4;... % front
        5, 6, 7, 8;... % back
        1, 4, 8, 5;... % top
        8, 4, 3, 7;... % right 
        1, 2, 6, 5;... % left
        2, 3, 7, 6;... % bottom
        ];
    
%--------- vertices and faces for connecting rods ------------    
% vertices for front rod
Vert_rod_front = [...
    drawParam.w/2, drawParam.lw/2, 0;...
    drawParam.w/2, -drawParam.lw/2, 0;...
    drawParam.w/2, 0, drawParam.lw/2;...
    drawParam.w/2, 0, -drawParam.lw/2;...
    drawParam.l+drawParam.w/2, drawParam.lw/2, 0;...
    drawParam.l+drawParam.w/2, -drawParam.lw/2, 0;...
    drawParam.l+drawParam.w/2, 0, drawParam.lw/2;...
    drawParam.l+drawParam.w/2, 0, -drawParam.lw/2;...
    ];
Face_rod_front = 8 + [...
        1, 2, 6, 5;... % x-y face
        3, 4, 8, 7;... % x-z face
    ];
% vertices for right rod
Vert_rod_right = Vert_rod_front*[0,1,0;1,0,0;0,0,1];
Face_rod_right = 16 + [...
        1, 2, 6, 5;... 
        3, 4, 8, 7;... 
    ];
% vertices for back rod
Vert_rod_back = Vert_rod_front*[-1,0,0;0,-1,0;0,0,1];
Face_rod_back = 24 + [...
        1, 2, 6, 5;... 
        3, 4, 8, 7;... 
    ];
% vertices for left rod
Vert_rod_left = Vert_rod_front*[0,-1,0;-1,0,0;0,0,1];
Face_rod_left = 32 + [...
        1, 2, 6, 5;... 
        3, 4, 8, 7;... 
    ];

%--------- vertices and faces for rotors ------------  
Vert_rotor = [];
for i=1:drawParam.N,
    Vert_rotor = [Vert_rotor; drawParam.r*cos(2*pi*i/drawParam.N), drawParam.r*sin(2*pi*i/drawParam.N), 0];
end
for i=1:drawParam.N,
    Vert_rotor = [Vert_rotor; drawParam.r/10*cos(2*pi*i/drawParam.N), drawParam.r/10*sin(2*pi*i/drawParam.N), 0];
end
Face_rotor = [];
for i=1:drawParam.N-1,
    Face_rotor = [Face_rotor; i, i+1, drawParam.N+i+1, drawParam.N+i];
end
Face_rotor = [Face_rotor; drawParam.N, 1, drawParam.N+1, 2*drawParam.N];

% front rotor
Vert_rotor_front = Vert_rotor + repmat([drawParam.w/2+drawParam.l+drawParam.r, 0, 0],2*drawParam.N,1);
Face_rotor_front = 40 + Face_rotor;
% right rotor
Vert_rotor_right = Vert_rotor + repmat([0, drawParam.w/2+drawParam.l+drawParam.r, 0],2*drawParam.N,1);
Face_rotor_right = 40 + 2*drawParam.N + Face_rotor;
% back rotor
Vert_rotor_back = Vert_rotor + repmat([-(drawParam.w/2+drawParam.l+drawParam.r), 0, 0],2*drawParam.N,1);
Face_rotor_back = 40 + 2*drawParam.N + 2*drawParam.N + Face_rotor;
% left rotor
Vert_rotor_left = Vert_rotor + repmat([0, -(drawParam.w/2+drawParam.l+drawParam.r), 0],2*drawParam.N,1);
Face_rotor_left = 40 + 2*drawParam.N + 2*drawParam.N + 2*drawParam.N + Face_rotor;

% collect all of the vertices for the quadrotor into one matrix
Vert_quad = [...
    Vert_center; Vert_rod_front; Vert_rod_right; Vert_rod_back;...
    Vert_rod_left; Vert_rotor_front; Vert_rotor_right;...
    Vert_rotor_back; Vert_rotor_left...
    ];
% collect all of the faces for the quadrotor into one matrix
Face_quad = [...
    Face_center; Face_rod_front; Face_rod_right; Face_rod_back;...
    Face_rod_left; Face_rotor_front; Face_rotor_right;...
    Face_rotor_back; Face_rotor_left...
    ];

myred = [1, 0, 0];
mygreen = [0, 1, 0];
myblue = [0, 0, 1];
myyellow = [1,1,0];

colors_quad = [...
    mygreen;... % fuselage front
    myblue;... % back
    myyellow;... % top
    myblue;... % right
    myblue;... % left
    myblue;... % bottom
    mygreen;... % rod front
    mygreen;... %
    mygreen;... % rod right
    mygreen;... %
    mygreen;... % rod back
    mygreen;... %
    mygreen;... % rod left
    mygreen;... %
    ];
for i=1:drawParam.N,
    colors_quad = [colors_quad; mygreen];   % front rotor
end
for i=1:drawParam.N,
    colors_quad = [colors_quad; myblue];  % right rotor
end
for i=1:drawParam.N,
    colors_quad = [colors_quad; myblue];  % left rotor
end
for i=1:drawParam.N,
    colors_quad = [colors_quad; myblue];  % back rotor
end

% end quadrotorVertFace

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [Vert_rails, Face_rails, colors_rails] = railVertFace(drawParam)
% Vertices = [x,y,z] position of each vertex
% Faces = defines how the vertices are connnected for form faces.  Each set
% of our vertices defines one face.

%-------vertices and faces for rails--------------
% vertices for right rail
Vert_rail_right = [...
    drawParam.w/2+drawParam.l, drawParam.l_rail/2, drawParam.w_rail;...
    drawParam.w/2+drawParam.l, drawParam.l_rail/2, 0;...
    drawParam.w/2+drawParam.l, -drawParam.l_rail/2, 0;...
    drawParam.w/2+drawParam.l, -drawParam.l_rail/2, drawParam.w_rail;...
    drawParam.w/2+drawParam.l+drawParam.w_rail, drawParam.l_rail/2, 0;...
    drawParam.w/2+drawParam.l+drawParam.w_rail, -drawParam.l_rail/2, 0;...
    ];
Face_rail_right = [...
        1, 2, 3, 4;... % x-y face
        5, 2, 3, 6;... % x-z face
    ];

% vertices for left rail
Vert_rail_left = Vert_rail_right*[-1,0,0;0,1,0;0,0,1];
Face_rail_left = 6 + [...
        1, 2, 3, 4;... % x-y face
        5, 2, 3, 6;... % x-z face
    ];

% collect vertices and faces
Vert_rails = [Vert_rail_right; Vert_rail_left];
Face_rails = [Face_rail_right; Face_rail_left];

colors_rails = [...
    drawParam.myyellow;...% right rail
    drawParam.myyellow;...%
    drawParam.myyellow;...% left rail
    drawParam.myyellow;...%
    ];

% end railVertFace

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [Vert_fov, Face_fov, colors_fov] = fovVertFace(pd,phi,theta,psi,drawParam,C)

%-------vertices and faces for camera field-of-view --------------
% vertices 
Vert_fov = pd*[...
    0, 0, 0;...
     tan( phi+C.cam_fov/2), tan(-theta+C.cam_fov/2), 1;...
     tan( phi+C.cam_fov/2), tan(-theta-C.cam_fov/2), 1;...
    -tan(-phi+C.cam_fov/2), tan(-theta+C.cam_fov/2), 1;...
    -tan(-phi+C.cam_fov/2), tan(-theta-C.cam_fov/2), 1;...
    ];
Vert_fov = rotateVert(Vert_fov,0,0,-psi);
Face_fov = [...
        1, 1, 2, 2;... % x-y face
        1, 1, 3, 3;... % x-y face
        1, 1, 4, 4;... % x-y face
        1, 1, 5, 5;... % x-y face
        2, 3, 5, 4;... % x-y face
    ];

colors_fov = [drawParam.myblue; drawParam.myblue; drawParam.myblue; drawParam.myblue; drawParam.myyellow];


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [X,Y,Z] = targetXYZ
  
  z = -0.01;
  pts = [...
       2,  0, z;...
       0,  2, z;...
       0,  1, z;...
      -2,  1, z;...
      -2, -1, z;...
       0, -1, z;...
       0, -2, z;...
       2,  0, z;...
      ];
  X = pts(:,1);
  Y = pts(:,2);
  Z = pts(:,3);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function handle = targetPlot(tn,te,td,tpsi,size,handle,color,mode)

  % get the target data points
  [X,Y,Z] = targetXYZ;
  
  % scale by size
  X = X*size;
  Y = Y*size;
  Z = Z*size;
  
  % rotate by tpsi
  [X,Y,Z] = rotateXYZ(X,Y,Z,0,0,tpsi);
  
  % translate by [tn, te, td]
  [X,Y,Z] = translateXYZ(X,Y,Z,[tn,te,td]);
  
  if isempty(handle),
%    handle = patch(X, Y, Z, color, 'EraseMode', mode);
    handle = patch(Y, X, -Z, color, 'EraseMode', mode);
  else
%    set(handle,'XData',X,'YData',Y,'ZData',Z);
    set(handle,'XData',Y,'YData',X,'ZData',-Z);
    drawnow
  end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function handle = targetCamPlot(tn,te,td,tpsi,size,handle,color,mode)

  % get the target data points
  [X,Y,Z] = targetXYZ;
  
  % scale by size
  X = X*size;
  Y = Y*size;
  Z = Z*size;
  
  % rotate by tpsi
  [X,Y,Z] = rotateXYZ(X,Y,Z,0,0,tpsi);
  
  % translate by [tn, te, td]
  [X,Y,Z] = translateXYZ(X,Y,Z,[tn,te,td]);
  
  if isempty(handle),
    handle = patch(X, Y, Z, color, 'EraseMode', mode);
  else
    set(handle,'XData',X,'YData',Y,'ZData',Z);
    drawnow
  end
  
%     function handle = drawCable(pn, pe, pd, handle, color, mode)

  