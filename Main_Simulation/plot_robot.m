%% plot_robot.m
%
% Description:
%   Plots the robot in its current configuration.
%   
% Inputs:
%   q: robot configuration, q = [x_cart; theta_pend];
%   params: a struct with many elements, generated by calling init_params.m
%   varargin: optional name-value pair arguments:
%       'new_fig': (default: false), if true, plot appears on new figure
%
% Outputs:
%   none
%
% Notes:
%   1) This code is written verbosely in the hope that it is clear how to
%   extend it to visualizing other more complex robots.

function plot_robot(q,params,varargin)
%% Parse input arguments
% Note: a simple robot plotting function doesn't need this, but I want to
% write extensible code, so I'm using "varargin" which requires input
% parsing. See the reference below:
%
% https://people.umass.edu/whopper/posts/better-matlab-functions-with-the-inputparser-class/

% Step 1: instantiate an inputParser:
p = inputParser;

% Step 2: create the parsing schema:
%      2a: required inputs:
addRequired(p,'bike_config', ...
    @(q) isnumeric(q) && size(q,1)==5 && size(q,2)==1);
addRequired(p,'bike_params', ...
    @(params) ~isempty(params));
%      2b: optional inputs:
addParameter(p, 'new_fig', false); % if true, plot will be on a new figure

% Step 3: parse the inputs:
parse(p, q, params, varargin{:});

%% Compute the corners of the bike's body, clockwise from top left corner
% First compute the cart's home position (q(1) = 0):
body.home.upp_left.x    = -0.5*params.model.geom.body.w;
body.home.upp_left.y    = 0.5*params.model.geom.body.h;

body.home.upp_right.x   = 0.6*params.model.geom.body.w;
body.home.upp_right.y   = 0.2*params.model.geom.body.h;

body.home.upp_mid.x   = 0;
body.home.upp_mid.y   = 0.2*params.model.geom.body.h;

body.home.low_right.x   = 1*params.model.geom.body.w;
body.home.low_right.y   = -0.3*params.model.geom.body.h;

body.home.low_left.x    = -0.5*params.model.geom.body.w;
body.home.low_left.y    = -0.3*params.model.geom.body.h;

body.home.corners = horzcat([body.home.upp_left.x; body.home.upp_left.y; 1],...
                            [body.home.upp_mid.x; body.home.upp_mid.y; 1],...
                            [body.home.upp_right.x; body.home.upp_right.y; 1],...
                            [body.home.low_right.x; body.home.low_right.y; 1],...
                            [body.home.low_left.x;  body.home.low_left.y; 1]);

% Compute the 4 corners of the body after undergoing planar
% translation + rotation as described by T_body:

bw_com_init_angle = params.model.geom.bw_com.theta;
bw_com_distance = params.model.geom.bw_com.l;
bw_fw_distance = params.model.geom.bw_fw.l;

x_bf = q(1);
y_bf = q(2);
theta_com = q(3);
theta_bw = q(4);
theta_fw = q(5);

g_wbf = [[cos(0), -sin(0), x_bf]; 
          [sin(0), cos(0), y_bf];
          [0, 0, 1]];
      
g_wbf_img_bot = [[cos(0), -sin(0), x_bf-0.2]; %-0.2
                [sin(0), cos(0), y_bf+0.4]; %0.4
                 [0, 0, 1]];
g_wbf_img_upp = [[cos(0), -sin(0), x_bf+0.35]; % 0.35
                [sin(0), cos(0), y_bf-0.25]; %-0.25
                 [0, 0, 1]];

g_rot = [[cos(bw_com_init_angle + theta_com), -sin(bw_com_init_angle + theta_com), 0]; 
         [sin(bw_com_init_angle + theta_com), cos(bw_com_init_angle + theta_com), 0];
         [0, 0, 1]];

g_trans = [[1, 0, bw_com_distance]; 
           [0, 1, 0];
           [0, 0, 1]];

g_bf_com = g_rot * g_trans;

% Need to rotate back the fixed angle amount to align the orientation with
% the world frame
g_rot_rev = [[cos(-bw_com_init_angle), -sin(-bw_com_init_angle), 0]; 
         [sin(-bw_com_init_angle), cos(-bw_com_init_angle), 0];
         [0, 0, 1]];

T_body = g_wbf*g_bf_com*g_rot_rev;

comX = T_body(1,3);
comY = T_body(2,3);


body.curr.corners = T_body*body.home.corners;

%% Compute the location of the wheels

% Back Wheel Transform
g_bf_bw = [[cos(theta_bw), -sin(theta_bw), 0]; 
           [sin(theta_bw), cos(theta_bw), 0];
           [0, 0, 1]];
T_w_bw = g_wbf * g_bf_bw;

% Front Wheel Transform
g_rot_com = [[cos(theta_com), -sin(theta_com), 0]; 
         [sin(theta_com), cos(theta_com), 0];
         [0, 0, 1]];
     
g_trans = [[1, 0, bw_fw_distance]; 
           [0, 1, 0];
           [0, 0, 1]];

g_rot_com_rev = [[cos(-theta_com), -sin(-theta_com), 0]; 
                 [sin(-theta_com), cos(-theta_com), 0];
                 [0, 0, 1]];
       
g_rot_fw = [[cos(theta_fw), -sin(theta_fw), 0]; 
            [sin(theta_fw), cos(theta_fw), 0];
            [0, 0, 1]];
        
g_bf_fw = g_rot_com*g_trans*g_rot_com_rev*g_rot_fw;

T_w_fw = g_wbf * g_bf_fw;

wheel.center = [0;0;1];
wheel.radius = [params.model.geom.wheel.r; 0; 1];                           

% Now compute the 4 corners of the legs after undergoing planar
% translation + rotation
wheel_fw.curr.center = T_w_fw*wheel.center;
wheel_fw.curr.radius = T_w_fw*wheel.radius;

wheel_bw.curr.center = T_w_bw*wheel.center;
wheel_bw.curr.radius = T_w_bw*wheel.radius;
       
%% Display the Bike
if p.Results.new_fig
    figure;
end

%Transform to rotate image
theta = -mod(round((180/pi)*theta_com),360);
tform = affine2d([ ...
    cosd(theta) sind(theta) 0;...
    -sind(theta) cosd(theta) 0; ...
    0 0 1]);

%load the image
[img, map, alphachannel] = imread("bike_imgs/bike_new.png");
%rotate the image
img =  imwarp(img,tform);
alphachannel = ~all(img == 0, 3);
%Move the image into position
T_img_bot = g_wbf_img_bot*g_bf_com*g_rot_rev;
T_img_upp = g_wbf_img_upp*g_bf_com*g_rot_rev;
img_x = [-0.5 + comX, 0.5 + comX];
img_y = [0.45 + comY, comY - 0.55];

%Create image
image(img_x,img_y,img,'AlphaData', alphachannel);
%Flip image so it's not upside down
set(gca,'YDir','normal')
grid on;
hold on;

%Uncomment this and comment image block to use shape for bike body instead
%of body
% fill(leg_bw.corners(1,:),leg_bw.corners(2,:),params.viz.colors.leg);
% fill(leg_fw.corners(1,:),leg_fw.corners(2,:),params.viz.colors.leg);

%Create wheels
circle(wheel_bw.curr.center(1),wheel_bw.curr.center(2),params.model.geom.wheel.r,params.viz.colors.tracers.wheels);
circle(wheel_fw.curr.center(1),wheel_fw.curr.center(2),params.model.geom.wheel.r,params.viz.colors.tracers.wheels);
segment(wheel_bw.curr.center(1), wheel_bw.curr.center(2),wheel_bw.curr.radius(1),wheel_bw.curr.radius(2), 'red');
segment(wheel_fw.curr.center(1), wheel_fw.curr.center(2),wheel_fw.curr.radius(1),wheel_fw.curr.radius(2), 'red');

if strcmp(params.sim.trick, 'Backflip')
    %Create ramp curve
    th = 3*pi/2:pi/50:(3*pi/2 +params.model.geom.ramp.theta);
    x_ramp = (params.model.geom.ramp.r + params.model.geom.wheel.r) * cos(th) + params.model.geom.ramp.center.x;
    y_ramp = (params.model.geom.ramp.r + params.model.geom.wheel.r) * sin(th) + params.model.geom.ramp.center.y;
    
    horizontal_line_x = x_ramp(end):0.1:x_ramp(end) + params.model.geom.ramp.width;
    horizontal_line_y = y_ramp(end).*ones(1,length(horizontal_line_x));
    
    vertical_line_y = fliplr(0:0.1:y_ramp(end));
    vertical_line_x = (x_ramp(end) + params.model.geom.ramp.width).*ones(1,length(vertical_line_y));

    
    bottom_x = fliplr(0:0.1:(x_ramp(end) + params.model.geom.ramp.width));
    bottom_y = zeros(1,length(bottom_x)); 
    
    plot(x_ramp, y_ramp, 'k');
    segment(x_ramp(end),y_ramp(end),x_ramp(end) + params.model.geom.ramp.width, y_ramp(end), 'k');
    segment(x_ramp(end) + params.model.geom.ramp.width,y_ramp(end),x_ramp(end) + params.model.geom.ramp.width, 0, 'k');
    
    fill([x_ramp,horizontal_line_x,vertical_line_x,bottom_x],[y_ramp,horizontal_line_y,vertical_line_y,bottom_y], 'k');
end

%Add Marker for CoM
 plot(comX, comY,'o','MarkerSize',10,...
     'MarkerFaceColor',params.viz.colors.wheels,...
     'MarkerEdgeColor',params.viz.colors.wheels);
 
%Add ground line
yline(0);


%Background image
%Mario Kart mode 
% I = imread('bike_imgs/mario_kart.jpg'); 
% h = image([5 15],[10 0],I); 
% uistack(h,'bottom')

I = imread('bike_imgs/background_bmx.jpg'); 
I = imresize(I, 1); 
h = image([-1 31],[5 -0.475],I); 
uistack(h,'bottom')


hold off;
if params.viz.tracking
    axis([x_bf+params.viz.x_axis_window(1),x_bf+params.viz.x_axis_window(2),params.viz.y_axis_window(1),params.viz.y_axis_window(2)]);
else
    axis(params.viz.axis_lims);
end

%Mario Kart mode 
% if q(1) > 8
% %     axis([5,15,0,10])
% %       axis([10-(q(1)-8)*(5/20),10+(q(1)-8)*(5/20),5-(q(1)-8)*(5/20),5+(q(1)-8)*(5/20)])
%       axis([10-(q(1)-8)*(5/20),10+(q(1)-8)*(5/20), -0.1 ,(q(1)-8)*(10/20)])
% else
%     if params.viz.tracking
%         axis([x_bf+params.viz.x_axis_window(1),x_bf+params.viz.x_axis_window(2),params.viz.y_axis_window(1),params.viz.y_axis_window(2)]);
%     else
%         axis(params.viz.axis_lims);
%     end
% end


daspect([1 1 1]) % no distortion

xlabel('$x$');
ylabel('$y$');

end


function circles = circle(x,y,r,c)
th = 0:pi/50:2*pi;
x_circle = r * cos(th) + x;
y_circle = r * sin(th) + y;
circles = plot(x_circle, y_circle);
fill(x_circle, y_circle, c)
end

function segments = segment(x1,y1,x2,y2,c)
x = [x1 x2];
y = [y1 y2];
segments = line(x,y,'Color',c);
end