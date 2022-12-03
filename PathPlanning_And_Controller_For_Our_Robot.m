
clear,close all;
clc;


obstacleMargin = 2.5;

initPose = [10;5;0]; 
startPoint = [10 5 0];
goalPoint  = [35 7 0];
tempGoalPoint = [35; 7];

initPose2 = [35;7;0]; 
startPoint2 = [35 7 0];
goalPoint2  = [5 35 pi];
tempGoalPoint2 = [5; 35];

%% Simulation setup
% Define Vehicle
R = (13*pi)/36;                        % Wheel radius [m]
L = 1.2;                        % Wheelbase [m]
%dd = DifferentialDrive(R,L);
bc = bicycleKinematics("WheelBase",L,"MaxSteeringAngle",R);
% Sample time and time array
sampleTime = 0.1;              % Sample time [s]
tVec = 0:sampleTime:20;        % Time array
% Initial conditions
% Initial pose (x y theta) is defined at the top to make running different
% paths easier in the testing phase
pose = zeros(3,numel(tVec));   % Pose matrix
pose(:,1) = initPose;
% Load map


% %Kode for varehus
% Varehus = binaryOccupancyMap(100,100,1);
% 
% %Definerer størrelse på x og y-akser
% occ = zeros(100,100);
% 
% %Definerer veggene til varehuset
% occ(1,:) = 1;
% occ(end,:) = 1;
% occ([1:20, 41:100],1) = 1;
% occ([1:100],end) = 1;
% 
% %Lager reolene
% %occ(1:25,[19:21 39:41 59:61 79:81]) = 1;
% occ(39:100,[1:10 45:55]) = 1;
% occ(1:100,[90:100]) = 1;
% 
% %Setter okkuperte områder inn i kartet
% setOccupancy(Varehus, occ)

%Kode for varehus
Varehus = binaryOccupancyMap(50,50,1);

%Definerer størrelse på x og y-akser
occ = zeros(50,50);

%Definerer veggene til varehuset
occ(1,:) = 1;
occ(end,:) = 1;
occ([1:10, 21:50],1) = 1;
occ([1:50],end) = 1;

%Lager reolene
%occ(1:25,[19:21 39:41 59:61 79:81]) = 1;
occ(25:50,[1:5 23:27]) = 1;
occ(1:50,[45:50]) = 1;

%Setter okkuperte områder inn i kartet
setOccupancy(Varehus, occ)

map = Varehus;


% Statevalidator blir brukt i planner vår
stateValidator = validatorOccupancyMap;
stateValidator.Map = map;
stateValidator.ValidationDistance = 0.1;



% Create lidar sensor 
lidar = LidarSensor;
lidar.sensorOffset = [0,0];
lidar.scanAngles = linspace(-pi/2,pi/2,51);
lidar.maxRange = 10;
% Create visualizer
viz = Visualizer2D;
viz.hasWaypoints = false;
viz.mapName = 'map';
%viz.objDetectorMaxRange = true;
attachLidarSensor(viz,lidar);

%% Path planning and following
% Create waypoints


% Hybrid A* planner from the Mobile robots toolbox
% here we can put a cost on different movements
planner = plannerHybridAStar(stateValidator,"ForwardCost",1,"ReverseCost",3, ...
    "DirectionSwitchingCost",6);    %,"MinTurningRadius",1.76);this was just used for testing



path = plan(planner,startPoint,goalPoint); % generate a path with lots of waypoints
inpath = path.States; % this is used for path optimization later

% path optimizing with built in function in the toolbox
options = optimizePathOptions;
options.MinTurningRadius = 1.76;
options.MaxPathStates = size(inpath,1)*3;
options.ObstacleSafetyMargin = obstacleMargin;

% here we run the path optimizing algorithm a few times too ensure we get a
% smooth path 
optpath1 = optimizePath(inpath,map,options);
optpath2 = optimizePath(optpath1,map,options);
optpath3 = optimizePath(optpath2,map,options);
optpath4 = optimizePath(optpath3,map,options);
optpath = optimizePath(optpath4,map,options);


% plot input path and optimized path
show(map)
hold on
quiver(inpath(:,1),inpath(:,2),cos(inpath(:,3)),sin(inpath(:,3)),0.1);
quiver(optpath(:,1),optpath(:,2),cos(optpath(:,3)),sin(optpath(:,3)),0.1);
legend("Input Path","Optimized Path")


% removing the last column from the optimized path so that we can use the x
% and y coordinatea for the pure pursuit controller
optpath(:,3)=[];
waypoints = optpath;


% Pure Pursuit Controller
controller = controllerPurePursuit; 
controller.Waypoints = waypoints;
controller.LookaheadDistance = 0.5;
controller.DesiredLinearVelocity = 2.67;
controller.MaxAngularVelocity = 3;

% Vector Field Histogram (VFH) for obstacle avoidance
vfh = controllerVFH;
vfh.DistanceLimits = [2.15 3];
vfh.NumAngularSectors = 36;
vfh.HistogramThresholds = [5 10];
vfh.RobotRadius = 0.9;
vfh.SafetyDistance = 0.6;
vfh.MinTurningRadius = 1.76;




% variables used in the simulationloop
r = rateControl(1/sampleTime);
hallo = true;
idx = 2;


% Simulation loop
while hallo
    % Get the sensor readings
    curPose = pose(:,idx-1);
    ranges = lidar(curPose);
        
    % Run the path following and obstacle avoidance algorithms
    [vRef,wRef,lookAheadPt] = controller(curPose);
    targetDir = atan2(lookAheadPt(2)-curPose(2),lookAheadPt(1)-curPose(1)) - curPose(3);
    steerDir = vfh(ranges,lidar.scanAngles,targetDir);    
    if ~isnan(steerDir) && abs(steerDir-targetDir) > 0.1
        wRef = 0.5*steerDir;
    end
    
    
    % Control the robot
    velB = [vRef;0;wRef];                   % Body velocities [vx;vy;w]
    vel = bodyToWorld(velB,curPose);  % Convert from body to world
    
    % Perform forward discrete integration step
    pose(:,idx) = curPose + vel*sampleTime; 
    
    % Update visualization
    viz(pose(:,idx),ranges)
    
    waitfor(r);
    idx = idx+1;
    tempCurPose = curPose;
    tempCurPose(end,:)=[];
    tempCurPose = round(tempCurPose,0);
    
    if tempCurPose == tempGoalPoint
        hallo = false;
    end
end


%% Second part of simulation
% the code is similar to the first part, i had to split the code up to
% simulate that the robot is in a fixed posistion for a few seconds while
% the manipulator retrives the pallet form the shelf before moving on to
% the drop of location

%clear all;

% %Kode for varehus
% Varehus = binaryOccupancyMap(100,100,1);
% 
% %Definerer størrelse på x og y-akser
% occ = zeros(100,100);
% 
% %Definerer veggene til varehuset
% occ(1,:) = 1;
% occ(end,:) = 1;
% occ([1:20, 41:100],1) = 1;
% occ([1:100],end) = 1;
% 
% %Lager reolene
% %occ(1:25,[19:21 39:41 59:61 79:81]) = 1;
% occ(39:100,[1:10 45:55]) = 1;
% occ(1:100,[90:100]) = 1;
% 
% %Setter okkuperte områder inn i kartet
% setOccupancy(Varehus, occ)
% %Kode for varehus
% Varehus = binaryOccupancyMap(50,50,1);
% 
% %Definerer størrelse på x og y-akser
% occ = zeros(50,50);
% 
% %Definerer veggene til varehuset
% occ(1,:) = 1;
% occ(end,:) = 1;
% occ([1:10, 21:50],1) = 1;
% occ([1:50],end) = 1;
% 
% %Lager reolene
% %occ(1:25,[19:21 39:41 59:61 79:81]) = 1;
% occ(19:50,[1:5 23:27]) = 1;
% occ(1:50,[45:50]) = 1;
% 
% %Setter okkuperte områder inn i kartet
% setOccupancy(Varehus, occ)
% map = Varehus;

stateValidator = validatorOccupancyMap;
stateValidator.Map = map;
stateValidator.ValidationDistance = 0.1;



% Create lidar sensor
lidar = LidarSensor;
lidar.sensorOffset = [0,0];
lidar.scanAngles = linspace(-pi/2,pi/2,51);
lidar.maxRange = 10;
% Create visualizer
viz = Visualizer2D;
viz.hasWaypoints = false;
viz.mapName = 'map';
%viz.objDetectorMaxRange = true;
attachLidarSensor(viz,lidar);

%% Simulation setup
% Define Vehicle
R = (13*pi)/36;                        % Wheel radius [m]
L = 1.2;                        % Wheelbase [m]
%dd = DifferentialDrive(R,L);
bc = bicycleKinematics("WheelBase",L,"MaxSteeringAngle",R);
% Sample time and time array
sampleTime2 = 0.1;              % Sample time [s]
tVec = 0:sampleTime2:9;        % Time array
% Initial conditions
% Initial pose (x y theta)
pose = zeros(3,numel(tVec));   % Pose matrix
pose(:,1) = initPose2;
% Load map


%% Path planning and following from pick up to place
% Create waypoints

planner2 = plannerHybridAStar(stateValidator,"ForwardCost",1,"ReverseCost",3, ...
    "DirectionSwitchingCost",5);%,"MinTurningRadius",1.76);


path2 = plan(planner2,startPoint2,goalPoint2);
inpath2 = path2.States;

options2 = optimizePathOptions;
options2.MinTurningRadius = 1.76;
options2.MaxPathStates = size(inpath2,1)*3;
options2.ObstacleSafetyMargin = obstacleMargin;

optpath3 = optimizePath(inpath2,map,options2);
optpath4 = optimizePath(optpath3,map,options2);
optpath5 = optimizePath(optpath4,map,options2);
%show(map)
hold on
quiver(inpath2(:,1),inpath2(:,2),cos(inpath2(:,3)),sin(inpath2(:,3)),0.1);
quiver(optpath5(:,1),optpath5(:,2),cos(optpath5(:,3)),sin(optpath5(:,3)),0.1);
legend("Input Path","Optimized Path")
hold off

% removing the last column from the optimized path so that we can use the x
% and y coordinatea for the pure pursuit controller
optpath5(:,3)=[];
waypoints = optpath5;
% Pure Pursuit Controller
controller2 = controllerPurePursuit; 
controller2.Waypoints = waypoints;
controller2.LookaheadDistance = 0.7;
controller2.DesiredLinearVelocity = 2.67;
controller2.MaxAngularVelocity = 3;

% Vector Field Histogram (VFH) for obstacle avoidance
% vfh = controllerVFH;
% vfh.DistanceLimits = [0.15 3];
% vfh.NumAngularSectors = 36;
% vfh.HistogramThresholds = [5 10];
% vfh.RobotRadius = 0.9;
% vfh.SafetyDistance = 0.6;
% vfh.MinTurningRadius = 1.76;

java.lang.Thread.sleep(5000);
%% Simulation loop 2 
r2 = rateControl(1/sampleTime2);

hei = true;
idx2 = 2;
%for idx2 = 2:numel(tVec) 

 while hei   

     
    % Get the sensor readings
    curPose2 = pose(:,idx2-1);
    ranges2 = lidar(curPose2);
        
    % Run the path following and obstacle avoidance algorithms
    [vRef,wRef,lookAheadPt] = controller2(curPose2);
    targetDir = atan2(lookAheadPt(2)-curPose2(2),lookAheadPt(1)-curPose2(1)) - curPose2(3);
    steerDir = vfh(ranges2,lidar.scanAngles,targetDir);    
    if ~isnan(steerDir) && abs(steerDir-targetDir) > 0.1
        wRef = 0.5*steerDir;
    end
    
    % Control the robot
    velB = [vRef;0;wRef];                   % Body velocities [vx;vy;w]
    vel = bodyToWorld(velB,curPose2);  % Convert from body to world
    
    % Perform forward discrete integration step
    pose(:,idx2) = curPose2 + vel*sampleTime2; 
    
    % Update visualization
    viz(pose(:,idx2),ranges2)
    
   
    waitfor(r2);

    
    idx2 = idx2+1;
    tempCurPose2 = curPose2;
    tempCurPose2(end,:)=[];
    tempCurPose2 = round(tempCurPose2,0);

    if tempGoalPoint2 == tempCurPose2
        hei = false;
    end
end
