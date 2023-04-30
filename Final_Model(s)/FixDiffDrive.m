%% Simulation parameters
dt = 0.1;   %step size
ts = 23;    %simulatioon time
t = 0:dt:ts;%time span

%vehicle(mobile robot) parameters (physical)
r = 0.2;   %radius of the wheel (fixed)
b = 0.5;    %distance b/t wheels (base)

%initial conditions
x0 = 2;
y0 = 2;
psi0 = 0;

eta0 = [x0;y0;psi0];
eta = zeros(3,numel(t));    % Pose matrix
eta(:,1) = eta0; %first element of the column matrix
%% Path planning
% Load map and inflate it by a safety distance
close all
load exampleMap
inflate(map,r); 
%inflate(map,radius) inflates each occupied position of the
% map by the radius given in meters.radius is rounded up to 
% the nearest cell equivalent based on the resolution of the map. 
% Every cell within the radius is set to true (1).

%Desired route or waypoints
waypoints = [2 2;
             4 4;
             4 8;
             11 8;
             11 11]
%% Pure Pursuit Controller
controller = controllerPurePursuit;
controller.Waypoints = waypoints;
controller.LookaheadDistance = 0.35;
controller.DesiredLinearVelocity = 0.75;
controller.MaxAngularVelocity = 1.5;

%% Create visualizer 
load exampleMap % Reload original (uninflated) map for visualization
viz = Visualizer2D;
viz.hasWaypoints = true;
viz.mapName = 'map';

%% Simulation loop starts here
rate = rateControl(1/dt);
for i = 1:numel(t)
     % Run the Pure Pursuit controller and convert output to wheel speeds
    [vRef,wRef] = controller(eta(:,i)); %[vel,angvel] = controller(pose) processes the vehicle's position and orientation, pose, and outputs the linear velocity, vel, and angular velocity, angvel.

    
    % Inverse loop = inverseKinematics(dd,vRef,wRef)
    % Calculates wheel speeds from linear and angular velocity
    wL = (vRef - wRef*b/2)/r; %wL = (v - w*obj.wheelBase/2)/obj.wheelRadius;
    wR = (vRef + wRef*b/2)/r; %wR = (v + w*obj.wheelBase/2)/obj.wheelRadius;
    

    % Compute the velocities, [v,w] = forwardKinematics(dd,wL,wR);
    % Calculates linear and angular velocity from wheel speeds
    v =0.5*r*(wL+wR); %v = 0.5*obj.wheelRadius*(wL+wR);
    w = (wR-wL)*r/b;  %w = (wR-wL)*obj.wheelRadius/obj.wheelBase;
    zetaB = [v;0;w]; % Body velocities [vx;vy;w]
     
    % Convert from body to world
    psi = eta(3,i);%current orientation in rad
    %jacobian matrix
    J_psi = [cos(psi) -sin(psi) 0;
             sin(psi) cos(psi) 0;
             0 0 1;];
    zeta = J_psi*zetaB;

     % Perform forward discrete integration step
    eta(:,i+1) = eta(:,i) + zeta*dt; 
    
    % Update visualization
    viz(eta(:,i+1),waypoints)
    waitfor(rate);
end
