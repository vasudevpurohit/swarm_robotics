%% vehicle parameters
t_width = 1.8;
w_radius = 0.2;
frameSize = t_width/0.5;
%% loading an image of the available 2D map and storing it in an array
I = imread('2D Map Image.jpg');
I = im2bw(I);

%flipping the pixel value so that white is 0 and black is 1
size = size(I);
x_size = size(1,1);
y_size = size(1,2);

for i = (1:1:x_size)
    for j = (1:1:y_size)
        if I(i,j) == 0
            I(i,j) = 1;
        else
            I(i,j) = 0;
        end
    end
end

%% creating a 2D-map with obstacles
map = binaryOccupancyMap(I,1);
inflate(map,t_width/2);
% show(map);

%% creating a roadmap based on the prm path planner
planner = mobileRobotPRM;
planner.Map = map;
planner.NumNodes = 200;
planner.ConnectionDistance = 100;

xy = findpath(planner,[50,10],[250,250]);

while isempty(xy) == 1
    update(planner);
    xy = findpath(planner,[50,10],[250,50]);
end

% show(planner);

%% kinematic model and controller for the differential drive robot
maxLinvel = 15;      %max linear speed of the vehicle
maxAng_wvel = maxLinvel/w_radius;   %max wheel speeds


%kinematic model for the robot
plantModel = differentialDriveKinematics;
plantModel.WheelRadius = w_radius;
plantModel.WheelSpeedRange = [-maxAng_wvel maxAng_wvel];
plantModel.TrackWidth = t_width;
plantModel.VehicleInputs = 'VehicleSpeedHeadingRate';

%pure pursuit controller
controller = controllerPurePursuit('DesiredLinearVelocity',maxLinvel,'MaxAngularVelocity',2,'Waypoints',xy,'LookaheadDistance',0.6);
%% simulating the navigation of the robot
current_pose = [xy(1,1) xy(1,2) pi/2];
total_disp = norm(xy(1,:)- xy(end,:));
vizrate = rateControl(10);

figure;

while  total_disp > 10
    %compute the inputs to the robot
    [v w] = controller(current_pose);
    vel   = derivative(plantModel, current_pose, [v w]);
    current_pose = current_pose + vel'*0.1;
    total_disp = norm(current_pose(1:2) - xy(end,:));
    

    show(map);
    plot(xy(:,1),xy(:,2),'k--o');
    hold all;
    
    plotTrVec = [current_pose(1:2) 0];
    plotRot = axang2quat([0 0 1 current_pose(3)]);
    plotTransforms(plotTrVec, plotRot, 'MeshFilePath', 'groundvehicle.stl', 'Parent', gca, "View","2D", "FrameSize", frameSize);

    
    waitfor(vizrate);
end