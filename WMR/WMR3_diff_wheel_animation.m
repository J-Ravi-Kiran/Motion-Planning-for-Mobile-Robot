clear; 
clc;

%%Simulation parameters
dt = 0.1;   %step size
ts = 100;    %simulatioon time
t = 0:dt:ts;%time span

%vehicle(mobile robot) parameters (physical)
a = 0.2;   %radius of the wheel (fixed)
d = 0.5;    %distance b/t wheels

%initial conditions
x0 = 0.5;
y0 = 0.5;
psi0 = pi/4;

eta0 = [x0;y0;psi0];

eta(:,1) = eta0; %first element of the column matrix

%loop starts here
for i = 1:length(t)
    psi = eta(3,i);%current orientation in rad
    %jacobian matrix
    J_psi = [cos(psi) -sin(psi) 0;
             sin(psi) cos(psi) 0;
             0 0 1;];
    %inputs
    omega_1 = 0.5; %left wheel angular velocity
    omega_2 = 0.2; %right wheel angular velocity

    omega = [omega_1;omega_2];

    %wheel configuration matrix
    W = [a/2 a/2;
         0 0;
         -a/(2*d) a/(2*d)];

    %velocity input commands
    zeta(:,i) = W*omega;
    eta_dot(:,i)=J_psi * zeta(:,i);

    eta(:,i+1) = eta(:,i) + dt * eta_dot(:,i); %Euler method
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
w = 2*d; % width of the mobile robot
%Mobile robot coordinates
mr_co=[-l/2 l/2 l/2 -l/2 -l/2;
       -w/2 -w/2 w/2 w/2 -w/2;];
figure
for i = 1:5:length(t) %animation starts here
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
    pause(0.01);
    hold off
end %animation ends here

