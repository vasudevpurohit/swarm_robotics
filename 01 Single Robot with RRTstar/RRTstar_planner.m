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
stateValidator.ValidationDistance = 5; %checks the validity after every 0.01m of motion
stateSpace.StateBounds = [map.XWorldLimits; map.YWorldLimits;[-pi pi]];

planner = plannerRRTStar(stateSpace,stateValidator);
planner.ContinueAfterGoalReached = true;
planner.MaxIterations = 80000;
planner.MaxConnectionDistance = 0.5;
planner.BallRadiusConstant = 1;
start = [50,10,0];
goal  = [250,250,0];

rng(100, 'twister')
[pthObj, solnInfo] = plan(planner,start,goal);

map.show;
hold on;
plot(solnInfo.TreeData(:,1),solnInfo.TreeData(:,2), '.-'); % tree expansion
plot(pthObj.States(:,1),pthObj.States(:,2),'r-','LineWidth',2); % draw path

%%
x = pthObj.States(:,1);
y = pthObj.States(:,2);
theta = pthObj.States(:,3);


%calculating the distance travelled at every point on the path
x_ref = [];
y_ref = [];
theta_ref = [];
dist(1,1) = 0;
for i = (1:1:length(x)-1)
    dist(i+1,1) = dist(i,1) + sqrt((x(i+1,1)-x(i,1))^2+(y(i+1,1)-y(i,1))^2);
end
dist_ref = (linspace(0,dist(end,1),50))';
x_ref = interp1(dist,x,dist_ref);
x_cs = spline(dist_ref,x_ref);
x_ref2 = ppval(x_cs,dist_ref);
y_ref = interp1(dist,y,dist_ref);
y_cs = spline(dist_ref,y_ref);
y_ref2 = ppval(y_cs,dist_ref);
theta_ref = interp1(dist,theta,dist_ref);
theta_cs = spline(dist_ref,theta_ref);
theta_ref2 = ppval(theta_cs,dist_ref);
% X_ref = [dist x_ref];
% Y_ref = [dist y_ref];
% Theta_ref = [dist theta_ref];

x_0 = x_ref2(1,1);
y_0 = y_ref2(1,1);
theta_0 = theta_ref2(1,1);

% calculating the curvature at each point
dx = gradient(x_ref2);
d2x = gradient(dx);
dy = gradient(y_ref2);
d2y = gradient(dy);
curvature = (dx.*d2y - dy.*d2x) ./(dx.^2+dy.^2).^(3/2);
%% accessing the timeseries samples
vehicle_pose = zeros(length(out.tout),3);
for i = (1:1:length(out.tout))
    vehicle_pose(i,(1:3)) = getdatasamples(out.currPose,i);
end
map.show;
hold on     
plot(x_ref2,y_ref2,'r-');
hold on
plot(vehicle_pose(:,1),vehicle_pose(:,2),'b-');
%%
% %% creating a controller to follow the poses found through RRT*
% maxLinvel = 15;      %max linear speed of the vehicle
% maxAng_wvel = maxLinvel/w_radius;   %max wheel speeds
% 
% 
% %kinematic model for the robot
% plantModel = differentialDriveKinematics;
% plantModel.WheelRadius = w_radius;
% plantModel.WheelSpeedRange = [-maxAng_wvel maxAng_wvel];
% plantModel.TrackWidth = t_width;
% plantModel.VehicleInputs = 'VehicleSpeedHeadingRate';
% 
% %pure pursuit controller
% controller = controllerPurePursuit('DesiredLinearVelocity',maxLinvel,'MaxAngularVelocity',2,'Waypoints',waypoints,'LookaheadDistance',0.05);
% %% simulating the navigation of the robot
% current_pose = [waypoints(1,1) waypoints(1,2) pi/2];
% total_disp = norm(waypoints(1,:)- waypoints(end,:));
% vizrate = rateControl(10);
% 
% figure;
% 
% while  total_disp > 1
%     %compute the inputs to the robot
%     [v w] = controller(current_pose);
%     vel   = derivative(plantModel, current_pose, [v w]);
%     current_pose = current_pose + vel'*0.05;
%     total_disp = norm(current_pose(1:2) - waypoints(end,:));
%     
% 
%     show(map);
%     plot(waypoints(:,1),waypoints(:,2),'k-');
%     hold all;
%     
%     plotTrVec = [current_pose(1:2) 0];
%     plotRot = axang2quat([0 0 1 current_pose(3)]);
%     plotTransforms(plotTrVec, plotRot, 'MeshFilePath', 'groundvehicle.stl', 'Parent', gca, "View","2D", "FrameSize", frameSize);
% 
%     
%     waitfor(vizrate);
% end