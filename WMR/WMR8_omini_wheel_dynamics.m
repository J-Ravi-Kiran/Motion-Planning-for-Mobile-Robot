clear;
clc;

%simulation parameters
dt=0.1; %step size
ts = 10; %simulation time
t = 0:dt:ts; %time span

%initial conditions
eta0 =[0;0;0];  %inital position and orientation of the vehicle
zeta0 = [0;0;0];%intial velocity of input commands

eta(:,1) = eta0;
zeta(:,1) = zeta0;

%vehicle(mobile robot) parameters (physical)
a = 0.2;   %radius of the wheel (fixed)
d = 0.5;    %distance b/t wheels

%robot parameters
m=10; %mass of the vehicle
Iz = 0.1; %inertia of the vehicle

xbc = 0; ybc = 0; %coordinates of mass centre

%wheel configuartion parameters
l=0.3;
phi1 = 90*pi/180; phi2 = 210*pi/180; phi3 = 330*pi/180;

%state propagation
for i = 1:length(t)
    u = zeta(1,i); v = zeta(2,i); r = zeta(3,i);

    %inertia matrix
    D = [m 0 -ybc*m;
         0 m xbc*m;
         -ybc*m xbc*m Iz+m*(xbc^2+ybc^2);];
    %other vector
    n_v = [-m*r*(v+xbc*r);
            m*r*(u-ybc*r);
            m*r*(xbc*u+ybc*v);];
    
    %Wheel input matrix (differential wheel drive)
    Gamma = [cos(phi1) cos(phi2) cos(phi3);
             sin(phi1) sin(phi2) sin(phi3);
             1 1 1;];

    %Wheel inputs (traction forces)
    F1 = 0;      
    F2 = -0.5;
    F3 = 0.5; %lateral 1,-0.5,-0.5 rotate 1,0.5,0.5 forward 0,-0.5,0.5
    kappa = [F1;F2;F3];

    %Input vector
    tau(:,i) = Gamma*kappa;

    %Jacobian matrix
    psi = eta(3,i);
    J_eta = [cos(psi) -sin(psi) 0;
             sin(psi) cos(psi) 0;
             0 0 1;];

    zeta_dot(:,i) = inv(D)*(tau(:,i)-n_v);
    zeta(:,i+1) = zeta(:,i) + dt*zeta_dot(:,i);

    eta(:,i+1) = eta(:,i) + dt*(J_eta*(zeta(:,i)+dt*zeta_dot(:,i))); %state update
end

%plotting 
figure
plot(t, eta(1,1:i), 'r-');      %for plotting x in m
hold on     
plot(t, eta(2,1:i), 'b--');     %y in m
plot(t, eta(3,1:i), 'm-.');     %t in s
legend('x, [m]','y, [m]','/psi, [rad]');
set(gca,'fontsize',24)
xlabel('t, [s]');
ylabel('/eta, [units]');

%animation (mobile robot motion animation)
l = 0.6; % length of mobile robot
w = 0.6; % width of the mobile robot
%Mobile robot coordinates
mr_co=[-l/2 l/2 l/2 -l/2 -l/2;
       -w/2 -w/2 w/2 w/2 -w/2;];

figure
for i = 1:length(t) %animation starts here
    psi = eta(3,i);
    R_psi=[cos(psi) -sin(psi);
           sin(psi)  cos(psi);];%rotation matrix
    v_pos=R_psi*mr_co;
    fill((v_pos(1,:)+eta(1,i)),v_pos(2,:)+eta(2,i),'g')
    hold on, grid on
    axis([-1 3 -1 3]), axis square
    plot(eta(1,1:i),eta(2,1:i),'b-');
    legend('MR','Path')
    set(gca,'fontsize',24)
    xlabel('x,[m]'); ylabel('y,[m]');
    pause(0.1);
    hold off
end %animation ends here


