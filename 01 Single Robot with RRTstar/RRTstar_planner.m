%% this code contains the implementation of the RRT* planning algorithm
%%with a direct differential drive single robot
clear all
%% vehicle parameters
t_width = 1.8;
w_radius = 0.2;
frameSize = t_width/0.5;

%% reading the environment image and creating its binary map
I = imread("2D Map Image.jpg");
I = im2bw(I);
size = size(I);
x_size = size(1,1);
y_size = size(1,2);

for i=(1:1:x_size)
    for j=(1:1:y_size)
        if I(i,j) == 1
            I(i,j)=0;
        else
            I(i,j)=1;
        end
    end
end

%% creating a 2D binary occupancy map
map = occupancyMap(I,1);
inflate(map,t_width/2);

%% setting the planner parameters
stateSpace = stateSpaceSE2;
stateValidator = validatorOccupancyMap(stateSpace);
stateValidator.Map = map;
stateValidator.ValidationDistance = 0.01; %checks the validity after every 0.01m of motion
stateSpace.StateBounds = [map.XWorldLimits; map.YWorldLimits;[-pi pi]];

planner = plannerRRTStar(stateSpace,stateValidator);
planner.ContinueAfterGoalReached = true;
planner.MaxIterations = 10000;
planner.MaxConnectionDistance = 0.3;
planner.BallRadiusConstant = 150;
start = [50,10,0];
goal  = [250,250,0];

rng(100, 'twister')
[pthObj, solnInfo] = plan(planner,start,goal);

% map.show;
% hold on;
% plot(solnInfo.TreeData(:,1),solnInfo.TreeData(:,2), '.-'); % tree expansion
% plot(pthObj.States(:,1),pthObj.States(:,2),'r-','LineWidth',2); % draw path

poses = [pthObj.States(:,1) pthObj.States(:,2) pthObj.States(:,3)];
waypoints = [poses(:,1) poses(:,2)];
%% creating a controller to follow the poses found through RRT*

%%