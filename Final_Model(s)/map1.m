myMap = binaryOccupancyMap(100,50,1)
walls = zeros(100,50);
walls(1,1:100) = 1; % Top wall
walls(50,:) = 1; % Bottom wall
walls(:,1) = 1; % Left wall
walls(:,100) = 1; % Right wall
%verical walls
walls(30:40,10) = 1;
walls(10:20,10) = 1;
walls(30:50,20) = 1;
walls(1:10,20) = 1;
walls(20:40,30) = 1;
walls(30:50,40) = 1;
walls(30:40,50) = 1;
walls(1:30,60) = 1;
walls(20:40,70) = 1;
walls(10:30,80) = 1;
walls(40:50,80) = 1;
walls(1:20,90) = 1;

%Horizontal walls
walls(20,10:30) = 1;
walls(10,20:40) = 1;
walls(10,50:60) = 1;
walls(40,50:70) = 1;
walls(30,60:70) = 1;
walls(10,70:80) = 1;
walls(40,80:90) = 1;
walls(30,90:100) = 1;

setOccupancy(myMap,[1 1],walls,"grid")
show(myMap)
inflateRadius =0.2
inflate(myMap,inflateRadius)
show(myMap)
%% 
map = binaryOccupancyMap(100,50,1)
setOccupancy(map,[1 1],walls,"grid");
setOccupancy(myMap,[1 1],walls,"grid")
show(myMap)