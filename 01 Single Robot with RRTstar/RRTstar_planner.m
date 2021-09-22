%% this code contains the implementation of the RRT* planning algorithm
%%with a direct differential drive single robot

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
map = binaryOccupancyMap(I,1);
inflate(map,t_width/2);

%% setting the planner parameters
