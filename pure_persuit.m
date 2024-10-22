%% EXAMPLE: Differential drive vehicle following waypoints using the 
% Pure Pursuit algorithm
%
% Copyright 2018-2019 The MathWorks, Inc.

%% Define Vehicle
R = 0.1;                % Wheel radius [m]
L = 0.5;                % Wheelbase [m]
dd = DifferentialDrive(R,L);

%% Simulation parameters
sampleTime = 0.1;               % Sample time [s]
tVec = 0:sampleTime:12;         % Time array

initPose = [0;0;0];             % Initial pose (x y theta)
pose = zeros(3,numel(tVec));    % Pose matrix
pose(:,1) = initPose;

% Define waypoints
waypoints = [0,0; 2,2; 4,2; 2,4; 0.5,3];
waypoints = [0,0; 1,0; 2,0; 3,0; 3,1; 3,2; 3,3; 2,3; 1,3; 0,3; 0,2; 0,1; 0,0; 1,0; 2,0];
%waypoints = [0,0; 0.1,0; 0.2,0; 0.3,0; 0.4,0;0.5,0; 0.5,0.1; 0.5,0.2; 0.5,0.3; 0.5,0.4; 0.5,0.5];

% Create visualizer
viz = Visualizer2D;
viz.hasWaypoints = true;

%% Pure Pursuit Controller
function [vRef, wRef] = PurePursuit(pose, waypoints, lookaheadDist, linearVel)

    currentPos = pose(1:2); 
    theta = pose(3); 

    distances = zeros(size(waypoints, 1), 1);

    % Loop to find distance of each waypoint from current position
    for i = 1:size(waypoints, 1)
        dx = waypoints(i, 1) - currentPos(1); 
        dy = waypoints(i, 2) - currentPos(2); 
        distances(i) = sqrt(dx^2 + dy^2);      
    end
    % Find closest distance
    [~, closest] = min(distances);
    
    % Finding the goal using lookahead Distance ahead of the closest
    % distance
    while norm(waypoints(closest,:) - currentPos') < lookaheadDist
        closest = closest + 1;
    end
   
    goal = waypoints(closest,:);

    goalRel = goal - currentPos';
    
    % Rotate the goal relative to the Robots frame
    goalRobotFrame = [cos(theta), sin(theta); ...
                      -sin(theta), cos(theta)] * goalRel';
    
    % Calculate the curvature
    y = goalRobotFrame(2);
    w = 2 * y / (lookaheadDist^2);
    wRef = w * linearVel;
    vRef = linearVel; 

end

%%controller = controllerPurePursuit;
%%controller = customPurePursuit;
Waypoints = waypoints;
LookaheadDistance = 0.4;
DesiredLinearVelocity = 0.95;
MaxAngularVelocity = 0.25;

%% Simulation loop
close all
r = rateControl(1/sampleTime);
for idx = 2:numel(tVec) 
    % Run the Pure Pursuit controller and convert output to wheel speeds
    [vRef,wRef] = PurePursuit(pose(:,idx-1), Waypoints, LookaheadDistance, DesiredLinearVelocity);
    [wL,wR] = inverseKinematics(dd,vRef,wRef);
    
    % Compute the velocities
    [v,w] = forwardKinematics(dd,wL,wR);
    velB = [v;0;w]; % Body velocities [vx;vy;w]
    vel = bodyToWorld(velB,pose(:,idx-1));  % Convert from body to world
    
    % Perform forward discrete integration step
    pose(:,idx) = pose(:,idx-1) + vel*sampleTime; 
    
    % Update visualization
    viz(pose(:,idx),waypoints)
    waitfor(r);
end