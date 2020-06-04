%% main.m
%
% Description:
%   Application entry point.
%
% Inputs: none
%
% Outputs: none
%
% Notes:

function main

%% Initialize environment
clear;
close all;
clc;

init_env();

%% Initialize parameters
params = init_params;

%% Visualize the robot in its initial state
x_IC = [params.sim.ICs.x_bf;
        params.sim.ICs.y_bf;
        params.sim.ICs.theta_com;
        params.sim.ICs.theta_bw;
        params.sim.ICs.theta_fw;
        params.sim.ICs.dx_com;
        params.sim.ICs.dy_com;
        params.sim.ICs.dtheta_com;
        params.sim.ICs.dtheta_bw;
        params.sim.ICs.dtheta_fw];

t_curr = 0;

% for the Controller 
prevError = 0;
prev_theta = 0;
eint = 0;
status = "NA";
tau = 0; %initial torque command 
alltau = []; 

% create a place for constraint forces populated in
% robot_dynamic_constraints function
F_calc = [0;0;-100;0];
tsim = [];
xsim = [];
xfin = [];

x_fw_ramp = 100; %x_position when the frontwheel hits the ramp
x_bw_ramp = 100; %x_position when the backwheel hits the ramp
fw_ang_compare = 100;  %y_position when the frontwheel leaves the ramp
bw_ang_compare = 100;  %y_position when the backwheel leaves the ramp

c_fw = 0;
c_bw = 0;

% Set integration options - mainly events
options = odeset('Events',@robot_events, 'RelTol',1e-3);

twrite = 0; 
dt = params.control.dt;

while twrite < params.sim.tfinal
    
    % time between this write and next write 
    tspan = [twrite, twrite+ dt]; 
    
    % analog plant
    %tspan_passive = t_curr:params.sim.dt:params.sim.tfinal;
    switch params.sim.trick
        case 'Backflip'
            [tseg, xseg, ~, ~, ie] = ode15s(@(t,x) robot_dynamics_constraints(t,x,tau), tspan, x_IC', options);
        case 'Wheelie'
            [tseg, xseg, ~, ~, ie] = ode45(@(t,x) robot_dynamics_constraints(t,x,tau), tspan, x_IC', options);
    end
    
    % state and sensor measurements at the time a read was made 
    tread = tseg(end) - params.control.delay;
    xread = interp1(tseg,xseg,tread); 
    
    % sensor measurement - assume perfect sensors
    theta_bw = xread(4);    % Angular Position of back wheel
    dtheta_bw = xread(9);   % Angular Velocity of back wheel
    theta_COM = xread(3);   % Angular Position of COM
    omega_est = (theta_bw-prev_theta)/params.sim.dt;    % Use encoder count to approximate angular velocity
    prev_theta = theta_bw;      % Store last used angular position


    currtime = tseg(end);
    %display(currtime)
    % compute control 
    switch params.sim.trick

        case 'Wheelie'
            % Wheelie %
            if (currtime<0.5)
                tau = -0.5;
            end

            if(currtime>=0.5&& currtime<0.7)
                tau = -2.5; 
            end

            if (currtime>=0.7)
                [tau_d,eint,prevError,status] = Controller(theta_COM,eint,prevError,status);
                tau = -Motor(tau_d,dtheta_bw); 
                %display(tau);
            end
 
            
        case 'Backflip' 
            %[tau_d,eint,prevError,status] = Controller(dtheta_bw,eint,prevError,status);
            %tau = -Motor(tau_d,dtheta_bw); 
            tau = -2.5;
            %{
            if theta_COM > 0
                tau = 0; 
                status = "On Ramp";
            end
            
            if abs(theta_COM)<0.0001
                status = "On Ground";
            end
            %}

            %tau = params.model.dyn.tau_bw * 0.05;
    end
    % Limit torque to feasible values
    if tau> 2.5
        tau = 2.5;
    elseif tau<-2.5
        tau = -2.5;
    end
            
    display(tau); 
    
    % update twrite and x_IC for next iteration 
    twrite = tseg(end); % set the current time to where the integration stopped
    x_IC = xseg(end,:); % set the initial condition to where the integration stopped
    
    counter = size(tseg,1);
    
    for i = 1:counter
        alltau = [alltau;tau]; % build up vector of all tau commands
    end

    % variables for plotting 
    % extract info from the integration
    tsim = [tsim;tseg]; % build up the time vector after each event
    xsim = [xsim;xseg]; % build up the calculated state after each event
    
    
    x_fw = x_IC(1) + params.model.geom.bw_fw.l;
    % if the simulation ended early, specify the new set of constraints
        
        switch params.sim.trick
            
            case 'Backflip'
                
                if  twrite < params.sim.tfinal %&& x_IC(1)>2.7 %FIXME this second condition should be necessary!
                    %but backflip constraints only work when this is true
                    %(x(1) jumps drastically while being integrated) find a
                    %way to correct this or add a better condition
                
                    switch params.sim.constraints

                        case ['flat_ground'] %both wheels are on the ground
                            if x_IC(1)+params.model.geom.bw_fw.l > x_fw_ramp
                                disp("FW is on the ramp!")
                                disp(tseg(end))
                                %disp(x_IC(1) + params.model.geom.bw_fw.l)
                                params.sim.constraints = ['fw_ramp'];
                            end
                        case ['fw_ramp']     %only the front wheel is on the ramp
                            if x_IC(1) > x_bw_ramp
                                disp("BW is on the ramp!")
                                disp(tseg(end))
                                params.sim.constraints = ['bw_ramp'];
                            end
                        case ['bw_ramp']     %both wheels are on the ramp
                            if x_IC(2)+params.model.geom.bw_fw.l > fw_ang_compare
                                disp("FW has left the ramp")
                                disp(tseg(end))
                                params.sim.constraints = ['fw_airborne'];
                            end
                        case ['fw_airborne']     %frontwheels leaves the ramp
                            if x_IC(2) > bw_ang_compare
                                disp("BW has left the ramp")
                                disp(tseg(end))
                                params.sim.constraints = ['bw_airborne'];
                            end
                            
                        case ['bw_airborne']
                             fw_h = x_IC(2) + params.model.geom.bw_fw.l*sin(x_IC(3));
                             if c_bw < 0 %(x_IC(2) < params.model.geom.wheel.r + params.model.dyn.collision_threshold)
                                 disp("BW Collision")
                                 [A_unilateral,~] = constraint_derivatives(x_IC,params); 
                                 A_col = A_unilateral(2,:); %add new constraint row to A matrix
                                 restitution = 1 + params.model.dyn.wheel_res; %restitiution being zero
                                 Minv_col = inv_mass_matrix(x_IC,params);
                                 x_IC(6:10) = x_IC(6:10) - (Minv_col*A_col'*inv(A_col*Minv_col*A_col')*diag(restitution)*A_col*x_IC(6:10)')';
                                % Often in a collision, the constraint forces will be violated
                                % immediately, rendering event detection useless since it requires a
                                % smoothly changing variable.  Therefore, we need to check the
                                % constraint forces and turn them off if they act in the wrong
                                % direction
                                if x_IC(2) > 0 && x_IC(2) < params.model.geom.wheel.r + 0.005%+params.model.dyn.collision_threshold
                                    disp('Put Backwheel constraint on again')
                                    params.sim.constraints = ['fw_off'];
                                end

                             elseif c_fw < 0 %(fw_h < params.model.geom.wheel.r + params.model.dyn.collision_threshold)
                                     disp("FW Collision")
                                     [A_unilateral,~] = constraint_derivatives(x_IC,params); 
                                     A_col = A_unilateral(3,:); %add new constraint row to A matrix
                                     restitution = 1 + params.model.dyn.wheel_res; %restitiution being zero
                                     Minv_col = inv_mass_matrix(x_IC,params);
                                     x_IC(6:10) = x_IC(6:10) - (Minv_col*A_col'*inv(A_col*Minv_col*A_col')*diag(restitution)*A_col*x_IC(6:10)')';
                                    % Often in a collision, the constraint forces will be violated
                                    % immediately, rendering event detection useless since it requires a
                                    % smoothly changing variable.  Therefore, we need to check the
                                    % constraint forces and turn them off if they act in the wrong
                                    % direction
                                    if fw_h > 0 && fw_h < params.model.geom.wheel.r + 0.005; %params.model.dyn.collision_threshold
                                        disp('Put frontwheel constraint on again')
                                        params.sim.constraints = ['flat_ground'];
                                    end

                             end
                            
                    end
                    
                end
                
            case 'Wheelie'
                
                if  twrite < params.sim.tfinal
                
                    switch params.sim.constraints

                         case ['flat_ground'] % both wheels are on the ground
                             if F_calc(3) > 0
                                disp("Changed Constraint!")
                                params.sim.constraints = ['fw_off']; % the front wheel is now off the ground    
                             end
                         case ['fw_off'] % both wheels are on the ground
                             if c_fw < 0
                                 
                                disp("Collision!")                           
                                [A_unilateral,~] = constraint_derivatives(x_IC,params); 
                                A_col = A_unilateral(3,:); %add new constraint row to A matrix
                                restitution = 1 + params.model.dyn.wheel_res; %restitiution being zero
                                Minv_col = inv_mass_matrix(x_IC,params);
                                % compute the change in velocity due to collision impulses
                                x_IC(6:10) = x_IC(6:10) - (Minv_col*A_col'*inv(A_col*Minv_col*A_col')*diag(restitution)*A_col*x_IC(6:10)')';
                                % Often in a collision, the constraint forces will be violated
                                % immediately, rendering event detection useless since it requires a
                                % smoothly changing variable.  Therefore, we need to check the
                                % constraint forces and turn them off if they act in the wrong
                                % direction
                                 if x_IC(3) > 0 && x_IC(3) < params.model.dyn.collision_threshold
                                     disp('Put frontwheel constraint on again')
                                     params.sim.constraints = ['flat_ground'];
                                 end
                             
                             end
                             
                    end
                          
                end
        end         
end

% transpose xsim_passive so that it is 5xN (N = number of timesteps):
 %% plotting and animation 
 figure;
 
 xplot = xsim';
  
 % plot the x and y position of the back wheel
 subplot(3,1,1), plot(tsim,xplot(1,:),'b-',...
                      tsim,xplot(2,:),'r-','LineWidth',2);
 lgd1 = legend({'x position back wheel','y position back wheel'},'Location','southwest');
 lgd1.FontSize = 10;
 xlabel('time')
 ylabel('position') 
 
 % plot the angle of the COM and the back wheel
 %subplot(2,1,2), plot(tsim,xplot(3,:),'b:',...
 %                     tsim,xplot(4,:),'r:','LineWidth',2);
 subplot(3,1,2), plot(tsim,xplot(3,:),'r:','LineWidth',2);
 lgd2 = legend({'angle COM','angle back wheel'},'Location','southwest');
 lgd2.FontSize = 10; 
 xlabel('time')
 ylabel('angle')
 
 % plot commanded tau values 
  subplot(3,1,3), plot(tsim,alltau,'r:','LineWidth',2);
 xlabel('time')
 ylabel('torque')
 pause(1); % helps prevent animation from showing up on the wrong figure
 
% Let's resample the simulator output so we can animate with evenly-spaced
% points in (time,state).
% 1) deal with possible duplicate times in tsim:
% (https://www.mathworks.com/matlabcentral/answers/321603-how-do-i-interpolate-1d-data-if-i-do-not-have-unique-values
tsim = cumsum(ones(size(tsim)))*eps + tsim;

anim_table = table(tsim,xsim);
[~,ia] = unique(anim_table.tsim);
anim_table_unique = anim_table(ia,:);

% 2) resample the duplicate-free time vector:
t_anim = 0:params.viz.dt:tsim(end);

% 3) resample the state-vs-time array:
x_anim = interp1(anim_table_unique.tsim, anim_table_unique.xsim, t_anim); %x_anim doesn't run in airborne
x_anim = x_anim'; % transpose so that xsim is 5xN (N = number of timesteps)
 
 animate_robot(x_anim(1:5,2:101),params,'trace_cart_com',false,...
     'trace_pend_com',false,'trace_pend_tip',false,'video',true);
 
 fprintf('Done passive simulation.\n');


%% robot_dynamics_constraints
function [dx] = robot_dynamics_constraints(t,x,tau)
% Robot Dynamics
% Description:
%   Computes the constraint forces: 
%       Fnow = inv(A*Minv*A')*(A*Minv*(Q-H) + Adotqdot)
%
%   Also computes the derivative of the state:
%       x_dot(1:5) = (I - A'*inv(A*A')*A)*x(6:10)
%       x_dot(6:10) = inv(M)*(Q - H - A'F)
%
% Inputs:
%   t: time (scalar)
%   x: the 10x1 state vector
%   params: a struct with many elements, generated by calling init_params.m
%
% Outputs:
%   dx: derivative of state x with respect to time.
% for convenience, define q_dot
    dx = zeros(numel(x),1);
    nq = numel(x)/2;    % assume that x = [q;q_dot];
    q_dot = x(nq+1:2*nq);


    Q = [0;0;0;tau;0];

    % find the parts that don't depend on constraint forces
    H = H_eom(x,params);
    Minv = inv_mass_matrix(x,params);
    [A_all,Hessian] = constraint_derivatives(x,params);

    switch params.sim.trick

        case 'Backflip'

            switch params.sim.constraints

                case ['flat_ground'] % both wheels on the ground
                    A = A_all([1,2,3,4],:);
                    Adotqdot = [q_dot'*Hessian(:,:,1)*q_dot;  % robot position x-constraint
                                q_dot'*Hessian(:,:,2)*q_dot;  % backwheel y-constraint
                                q_dot'*Hessian(:,:,3)*q_dot;  % frontwheel y-constraint
                                q_dot'*Hessian(:,:,4)*q_dot]; % frontwheel rotation constraint

                    Fnow = (A*Minv*A')\(A*Minv*(Q - H) + Adotqdot);
                    dx(1:nq) = (eye(nq) - A'*((A*A')\A))*x(6:10);
                    dx(nq+1:2*nq) = Minv*(Q - H - A'*Fnow) - A'*((A*A')\A)*q_dot/params.sim.dt;
                    x_fw_ramp = (x(1) + params.model.geom.bw_fw.l) - params.model.geom.ramp.center.x;
                    %if x_fw_ramp > -0.03 && x_fw_ramp < 0
                    disp("Robot dynamics x_fw")
                    disp(x_fw_ramp)
                    %end

                case ['fw_ramp'] % front wheel is on the ramp
                    A = A_all([1,2,6],:);
                    Adotqdot = [q_dot'*Hessian(:,:,1)*q_dot;  % robot position x-constraint
                                q_dot'*Hessian(:,:,2)*q_dot;  % backwheel flat ground constraint
                                q_dot'*Hessian(:,:,6)*q_dot]; % frontwheel ramp constraint

                    Fnow = (A*Minv*A')\(A*Minv*(Q - H) + Adotqdot);
                    dx(1:nq) = (eye(nq) - A'*((A*A')\A))*x(6:10);
                    dx(nq+1:2*nq) = Minv*(Q - H - A'*Fnow) - A'*((A*A')\A)*q_dot/params.sim.dt;
                    x_bw_ramp = x(1) - params.model.geom.ramp.center.x;
                    disp("FW on the ramp")

                case ['bw_ramp'] % both wheels on the ramp
                    A = A_all([1,5,6],:);
                    Adotqdot = [q_dot'*Hessian(:,:,1)*q_dot;  % robot position x-constraint
                                q_dot'*Hessian(:,:,5)*q_dot;  % backwheel ramp constraint
                                q_dot'*Hessian(:,:,6)*q_dot]; % frontwheel ramp constraint

                    Fnow = (A*Minv*A')\(A*Minv*(Q - H) + Adotqdot);
                    dx(1:nq) = (eye(nq) - A'*((A*A')\A))*x(6:10);
                    dx(nq+1:2*nq) = Minv*(Q - H - A'*Fnow) - A'*((A*A')\A)*q_dot/params.sim.dt;

                    %vertical height between fw and bw on the ramp
                    %height_fw_bw = params.model.geom.body.w*cos(0.5*acos(1-(params.model.geom.body.w^2/(2*params.model.geom.ramp.r^2))));
    %                 height_fw_bw = params.model.geom.bw_fw.l*cos(x(3));
    %                 y_fw_top = x(2) - (params.model.geom.ramp.h - height_fw_bw);%(params.model.geom.ramp.y - height_fw_bw);

                    % Find the angle from the start of the ramp to the ramp
                    % center to the fw center

                    % Point C
                    x_fw = x(1) + (params.model.geom.bw_fw.l)*cos(x(3));
                    y_fw = x(2) + (params.model.geom.bw_fw.l)*sin(x(3));

                    % Point B
                    ramp_center_x = params.model.geom.ramp.center.x;
                    ramp_center_y = params.model.geom.ramp.r;

                    % Point A
                    ramp_start_x = params.model.geom.ramp.center.x;
                    ramp_start_y = params.model.geom.wheel.r;

                    vec_BA = [ramp_start_x - ramp_center_x, ramp_start_y - ramp_center_y];

                    vec_BC = [x_fw - ramp_center_x, y_fw - ramp_center_y];

                    mag_BA = sqrt(vec_BA(1)^2 + vec_BA(2)^2);
                    mag_BC = sqrt(vec_BC(1)^2 + vec_BC(2)^2);

                    dot_vecs = dot(vec_BA, vec_BC);
                    mags = mag_BA*mag_BC;

                    ang_ramp_fw = acos(dot_vecs / mags);

                    fw_ang_compare = ang_ramp_fw - params.model.geom.ramp.theta;

                case ['fw_airborne'] % front wheel leves the ramp
                    A = A_all([1,5],:);
                    Adotqdot = [q_dot'*Hessian(:,:,1)*q_dot;  % robot position x-constraint
                                q_dot'*Hessian(:,:,5)*q_dot];  % backwheel ramp constraint

                    Fnow = (A*Minv*A')\(A*Minv*(Q - H) + Adotqdot);
                    dx(1:nq) = (eye(nq) - A'*((A*A')\A))*x(6:10);
                    dx(nq+1:2*nq) = Minv*(Q - H - A'*Fnow) - A'*((A*A')\A)*q_dot/params.sim.dt;

                    % y_bw_top = x(2) - params.model.geom.ramp.center.y;

                    % Find the angle from the start of the ramp to the ramp
                    % center to the fw center

                    % Point C - the state vector x(1) and x(2)

                    % Point B
                    ramp_center_x = params.model.geom.ramp.center.x;
                    ramp_center_y = params.model.geom.ramp.r;

                    % Point A
                    ramp_start_x = params.model.geom.ramp.center.x;
                    ramp_start_y = params.model.geom.wheel.r;

                    vec_BA = [ramp_start_x - ramp_center_x, ramp_start_y - ramp_center_y];
                    vec_BC = [x(1) - ramp_center_x, x(2) - ramp_center_y];

                    mag_BA = sqrt(vec_BA(1)^2 + vec_BA(2)^2);
                    mag_BC = sqrt(vec_BC(1)^2 + vec_BC(2)^2);

                    dot_vecs = dot(vec_BA, vec_BC);
                    mags = mag_BA*mag_BC;

                    ang_ramp_bw = acos(dot_vecs / mags);

                    bw_ang_compare = ang_ramp_bw - params.model.geom.ramp.theta;

                case ['bw_airborne'] % both wheels leaves the ramp
                    dx(1:nq) = eye(nq)*x(6:10);
                    dx(nq+1:2*nq) = Minv*(Q - H); %- A'*((A*A')\A)*q_dot/params.sim.dt;
                    c_bw = x(2) - params.model.geom.wheel.r;
                    c_fw = x(2) + params.model.geom.bw_fw.l*sin(x(3)) - params.model.geom.wheel.r;

                case ['fw_off'] % only the back wheel is on the ground
                     A = A_all([1,2],:);
                     Adotqdot = [q_dot'*Hessian(:,:,1)*q_dot; % robot position x-constraint
                                 q_dot'*Hessian(:,:,2)*q_dot]; % backwheel y-constraint
                     Fnow = (A*Minv*A')\(A*Minv*(Q - H) + Adotqdot);
                     dx(1:nq) = (eye(nq) - A'*((A*A')\A))*x(6:10);
                     dx(nq+1:2*nq) = Minv*(Q - H - A'*Fnow) - A'*((A*A')\A)*q_dot/params.sim.dt;
                     c_fw = params.model.geom.bw_fw.l*sin(x(3));

            end

        case 'Wheelie'

            switch params.sim.constraints

                case ['flat_ground'] % both wheels on the ground
                    A = A_all([1,2,3,4],:);
                    Adotqdot = [q_dot'*Hessian(:,:,1)*q_dot;  % robot position x-constraint
                                q_dot'*Hessian(:,:,2)*q_dot;  % backwheel y-constraint
                                q_dot'*Hessian(:,:,3)*q_dot;  % frontwheel y-constraint
                                q_dot'*Hessian(:,:,4)*q_dot]; % frontwheel rotation constraint

                    Fnow = (A*Minv*A')\(A*Minv*(Q - H) + Adotqdot);
                    dx(1:nq) = (eye(nq) - A'*((A*A')\A))*x(6:10);
                    dx(nq+1:2*nq) = Minv*(Q - H - A'*Fnow) - A'*((A*A')\A)*q_dot/params.sim.dt;
                    F_calc = Fnow;


                case ['fw_off'] % only the back wheel is on the ground
                     A = A_all([1,2],:);
                     Adotqdot = [q_dot'*Hessian(:,:,1)*q_dot; % robot position x-constraint
                                 q_dot'*Hessian(:,:,2)*q_dot]; % backwheel y-constraint
                     Fnow = (A*Minv*A')\(A*Minv*(Q - H) + Adotqdot);
                     dx(1:nq) = (eye(nq) - A'*((A*A')\A))*x(6:10);
                     dx(nq+1:2*nq) = Minv*(Q - H - A'*Fnow) - A'*((A*A')\A)*q_dot/params.sim.dt;
                     c_fw = params.model.geom.bw_fw.l*sin(x(3)); % - params.model.geom.wheel.r;
                     
            end

    end
end
 
%% Event handling Function: robot_events
 function [value,isterminal,direction] = robot_events(~,~)
    
    % F_calc comes from the robot_dynamics_constraints function above
     
    % MATLAB Documentation
    % value, isterminal, and direction are vectors whose ith element corresponds to the ith event function:
    % value(i) is the value of the ith event function.
    % isterminal(i) = 1 if the integration is to terminate at a zero of this event function. Otherwise, it is 0.
    % direction(i) = 0 if all zeros are to be located (the default). A value of +1 locates only zeros where the event function is increasing, and -1 locates only zeros where the event function is decreasing.     
switch params.sim.trick 
    
    case 'Backflip'
 
         switch params.sim.constraints

             case ['flat_ground']
                 value = x_fw_ramp; % use the value corresponding to the front wheel constraint
                 isterminal = 1; % tell ode45 to terminate if the event has occured
                 direction = 1; % tell ode45 to look for a positive constraint force as the event
                 disp("Robot events x_fw")
                 disp(x_fw_ramp)

             case ['fw_ramp']
                 
                 value = x_bw_ramp;
                 isterminal = 1;
                 direction = 0;

             case ['bw_ramp']
                 
                 %disp(fw_ang_compare);
                 
                 value = fw_ang_compare;
                 isterminal = 1;
                 direction = 0;

             case ['fw_airborne']
                 value = bw_ang_compare;
                 isterminal = 1;
                 direction = 0;
                 
             case ['bw_airborne']
                 value = [c_bw,c_fw];
                 isterminal = [1,1];
                 direction = [-1,-1];
                 
             case ['fw_off']
                  value = c_fw;
                  isterminal = 1;
                  direction = -1;
         end
         
    case 'Wheelie'
      
      switch params.sim.constraints
          
          case ['flat_ground']
                 value = F_calc(3); % use the value corresponding to the front wheel constraint
                 isterminal = 1; % tell ode45 to terminate if the event has occured
                 direction = 1; % tell ode45 to look for a positive constraint force as the event
          
          case ['fw_off']
                value = 0; %c_fw
                isterminal = 1;
                direction = -1;
      end
 end
 
 end % end of robot_events


%% Control the unstable equilibrium with LQR
% A = upright_state_matrix(params);
% B = upright_input_matrix(params);
% 
% % numerical verify the rank of the controllability matrix:
% Co = [B, A*B, (A^2)*B, (A^3)*B];
% fprintf('rank(Co) = %d.\n',rank(Co));
% 
% % control design: weights Q and R:
% Q = diag([5000,100,1,1]);    % weight on regulation error
% R = 1;                  % weight on control effort
% 
% % compute and display optimal feedback gain matrix K:
% K = lqr(A,B,Q,R);
% buf = '';
% for i = 1:size(K,2)
%     buf = [buf,'%5.3f '];
% end
% buf = [buf,'\n'];
% fprintf('LQR: K = \n');
% fprintf(buf,K');
% 
% % we could ask what are the eigenvalues of the closed-loop system:
% eig(A - B*K)
% 
% % add K to our struct "params":
% params.control.inverted.K = K;
% 
% % Simulate the robot under this controller:
% tspan_stabilize = 0:params.sim.dt:5;
% [tsim_stabilize, xsim_stabilize] = ode45(@(t,x) robot_dynamics(...
%     t,x,0,params,'controller','stabilize'),...
%     tspan_stabilize, x_IC');
% 
% % tranpose xsim_passive so that it is 4xN (N = number of timesteps):
% xsim_stabilize = xsim_stabilize'; % required by animate_robot.m
% 
% figure;
% subplot(2,1,1), plot(tsim_stabilize,xsim_stabilize(1,:),'b-',...
%                      tsim_stabilize,xsim_stabilize(2,:),'r-','LineWidth',2);
% subplot(2,1,2), plot(tsim_stabilize,xsim_stabilize(3,:),'b:',...
%                      tsim_stabilize,xsim_stabilize(4,:),'r:','LineWidth',2);
% pause(1); % helps prevent animation from showing up on the wrong figure
% 
% 
% animate_robot(xsim_stabilize(1:2,:),params,'trace_cart_com',true,...
%     'trace_pend_com',true,'trace_pend_tip',true,'video',true);
% fprintf('Done passive simulation.\n');

end
