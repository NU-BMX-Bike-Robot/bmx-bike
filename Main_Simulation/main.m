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
prevError = 0; % PID error derivative term
prev_theta = 0; % Previous encoder count (Used for calculating speed)
eint = 0; % PID error integral term
status = "NA"; %Used for debugging purposes for controller
tau = 0; %initial torque command 
voltage_d = 0; %desired voltage input to motor
alltau = []; % used for graphing commanded torque
alldtheta_bw = []; %Used for graphing back wheel velocity

% create a place for constraint forces populated in
% robot_dynamic_constraints function
F_calc = [0;0;-100;0];
tsim = [];
xsim = [];

%Variables for event triggerring
x_bw_ramp = 100; %x_position when the backwheel hits the ramp
fw_ang_compare = 100;  %y_position when the frontwheel leaves the ramp
bw_ang_compare = 100;  %y_position when the backwheel leaves the ramp
c_fw = 0; %Constraint equation of front wheel
c_bw = 0; %Constraint equation of back wheel


% Set integration options - mainly events
options = odeset('Events',@robot_events, 'RelTol',1e-3);

twrite = 0; 
dt = params.control.dt;

%% Discretized controller structure
while twrite < params.sim.tfinal 
    
    % time between this write and next write 
    tspan = [twrite, twrite+ dt]; 
    
    
    % analog plant
    %Using ode15s for Bacfklip trick due to better performance at changing
    %constraints
    switch params.sim.trick
        case 'Backflip'
            [tseg, xseg, ~, ~, ~] = ode15s(@(t,x) robot_dynamics_constraints(t,x,tau), tspan, x_IC', options);
        case 'Wheelie'
            [tseg, xseg, ~, ~, ~] = ode45(@(t,x) robot_dynamics_constraints(t,x,tau), tspan, x_IC', options);
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
    % compute control 
    switch params.sim.trick

        case 'Wheelie'
            
            %Initial velocity
            if (currtime<0.5)
                tau = -0.5;
            end
            %Kick-up 
            if(currtime>=0.5&& currtime<0.7)
                tau = -2.5; 
            end
            %Control to balance on back wheel
            if (currtime>=0.7)
                [voltage_d,eint,prevError,status] = Controller(theta_COM,eint,prevError,status);
                tau = -Motor(voltage_d,dtheta_bw); 
            end
 
            
        case 'Backflip' 
            %Speed control for backflip
            [voltage_d,eint,prevError,status] = Controller(dtheta_bw,eint,prevError,status); %FIXME remove status
            tau = Motor(voltage_d,dtheta_bw); 
            
    end
    % Limit torque to feasible values
    if tau> 2.5
        tau = 2.5;
    elseif tau<-2.5
        tau = -2.5;
    end
            
    %display(tau); 
    
    % update twrite and x_IC for next iteration 
    twrite = tseg(end); % set the current time to where the integration stopped
    x_IC = xseg(end,:); % set the initial condition to where the integration stopped
    
    counter = size(tseg,1);
    
    for i = 1:counter
        alltau = [alltau;tau]; % build up vector of all tau commands for plotting
        alldtheta_bw = [alldtheta_bw;dtheta_bw]; %build up vector for back wheel velocites for plotting
    end

    % variables for plotting 
    % extract info from the integration
    tsim = [tsim;tseg]; % build up the time vector after each event
    xsim = [xsim;xseg]; % build up the calculated state after each event
    
    %Front wheel position to check to turn off controller
    x_fw = x_IC(1) + params.model.geom.bw_fw.l;
   
        
        switch params.sim.trick
            
            case 'Backflip' %Backflip control will be only enable before frontwheel hits the ramp
                
                if  twrite < params.sim.tfinal 
                
                    switch params.sim.constraints
                        case ['flat_ground'] %both wheels are on the ground
                            if x_IC(1)+params.model.geom.bw_fw.l > params.model.geom.ramp.center.x
                                disp("FW is on the ramp, shut off control!")
%                                 disp(tseg(end))
                                params.sim.constraints = ['fw_ramp'];
                                t_curr = twrite; 
                                break %Go to simulation without control
                            
                            end
                        
                    end
                    
                end
                
            case 'Wheelie'
                
                if  twrite < params.sim.tfinal
                
                    switch params.sim.constraints

                         case ['flat_ground'] % both wheels are on the ground
                             %Check if constraint force of front wheel is
                             %positive to turn off front wheel constraint
                             if F_calc(3) > 0
                                disp("Changed Constraint!")
                                params.sim.constraints = ['fw_off']; % the front wheel is now off the ground    
                             end
                         case ['fw_off'] % both wheels are on the ground
                             %Check if constraint equation of front wheel
                             %has been violated
                             if c_fw < 0 
                                disp("Collision!")                           
                                [A_unilateral,~] = constraint_derivatives(x_IC,params); 
                                A_col = A_unilateral(3,:); %add new constraint row to A matrix
                                restitution = 1 + params.model.dyn.wheel_res;
                                Minv_col = inv_mass_matrix(x_IC,params);
                                % compute the change in velocity due to collision impulses
                                x_IC(6:10) = x_IC(6:10) - (Minv_col*A_col'*inv(A_col*Minv_col*A_col')*diag(restitution)*A_col*x_IC(6:10)')';
                                %Check theta COM to add flat ground
                                %constraint again
                                 if x_IC(3) > 0 && x_IC(3) < params.model.dyn.collision_threshold
                                     disp('Put frontwheel constraint on again')
                                     params.sim.constraints = ['flat_ground'];
                                 end
                             
                             end
                             
                    end
                          
                end
        end    
            
end


%% Simulation without controller structure for backflip
if params.sim.trick == "Backflip"
    while params.sim.tfinal - t_curr > params.sim.dt
        
    tspan_passive = t_curr:params.sim.dt:params.sim.tfinal;
    tau = 0; %turn off torque for ramp
    
    [tseg, xseg, ~, ~, ie] = ode15s(@(t,x) robot_dynamics_constraints(t,x,tau), tspan_passive, x_IC', options);
    
    

    % extract info from the integration
    tsim = [tsim;tseg]; % build up the time vector after each event
    xsim = [xsim;xseg]; % build up the calculated state after each event
    
    xfin = xseg(end,:);
    
    t_curr = tsim(end); % set the current time to where the integration stopped
    x_IC = xsim(end,:); % set the initial condition to where the integration stopped
    
    x_fw = x_IC(1) + params.model.geom.bw_fw.l;
    % if the simulation ended early, specify the new set of constraints
        
        switch params.sim.trick
            
            case 'Backflip'
                
                if  params.sim.tfinal - tseg(end) > params.sim.dt 
                
                    switch params.sim.constraints
                        
                        case ['fw_ramp']     %only the front wheel is on the ramp
                            disp("BW is on the ramp!")
                            disp(tseg(end))
                            params.sim.constraints = ['bw_ramp'];
                        case ['bw_ramp']     %both wheels are on the ramp
                            disp("FW has left the ramp")
                            disp(tseg(end))
                            params.sim.constraints = ['fw_airborne'];
                        case ['fw_airborne']     %frontwheels leaves the ramp
                            disp("BW has left the ramp")
                            disp(tseg(end))
                            params.sim.constraints = ['bw_airborne'];
                            
                        case ['bw_airborne']
                             fw_h = x_IC(2) + params.model.geom.bw_fw.l*sin(x_IC(3));
                             if(x_IC(2) < params.model.geom.wheel.r + params.model.dyn.collision_threshold)
                                 disp("BW Collision")
                                 [A_unilateral,~] = constraint_derivatives(x_IC,params); 
                                 A_col = A_unilateral(2,:); %add new constraint row to A matrix
                                 restitution = 1 + params.model.dyn.wheel_res; 
                                 Minv_col = inv_mass_matrix(x_IC,params);
                                 x_IC(6:10) = x_IC(6:10) - (Minv_col*A_col'*inv(A_col*Minv_col*A_col')*diag(restitution)*A_col*x_IC(6:10)')';
                                % Check back wheel constraint equation
                                % during collision to put back the back
                                % wheel constraint again
                                if x_IC(2) > 0 && x_IC(2) < params.model.geom.wheel.r + 0.005
                                    disp('Put Backwheel constraint on again')
                                    params.sim.constraints = ['fw_off'];
                                end

                             elseif(fw_h < params.model.geom.wheel.r + params.model.dyn.collision_threshold)
                                     disp("FW Collision")
                                     [A_unilateral,~] = constraint_derivatives(x_IC,params); 
                                     A_col = A_unilateral(3,:); %add new constraint row to A matrix
                                     restitution = 1 + params.model.dyn.wheel_res;
                                     Minv_col = inv_mass_matrix(x_IC,params);
                                     x_IC(6:10) = x_IC(6:10) - (Minv_col*A_col'*inv(A_col*Minv_col*A_col')*diag(restitution)*A_col*x_IC(6:10)')';
                                    % Check front wheel constraint equation
                                    % during collision to put back the
                                    % front wheel constraint again
                                    if fw_h > 0 && fw_h < params.model.geom.wheel.r + 0.005 
                                        disp('Put frontwheel constraint on again')
                                        params.sim.constraints = ['bw_off'];
                                    end
                                    
                              elseif(fw_h < params.model.geom.wheel.r + params.model.dyn.collision_threshold && x_IC(2) < params.model.geom.wheel.r + params.model.dyn.collision_threshold)
                                     disp("two wheel Collision")
                                     [A_unilateral,~] = constraint_derivatives(x_IC,params); 
                                     A_col = A_unilateral([2,3],:); %add new constraint row to A matrix
                                     restitution = [1 + params.model.dyn.wheel_res;
                                         1 + params.model.dyn.wheel_res]; 
                                     Minv_col = inv_mass_matrix(x_IC,params);
                                     x_IC(6:10) = x_IC(6:10) - (Minv_col*A_col'*inv(A_col*Minv_col*A_col')*diag(restitution)*A_col*x_IC(6:10)')';
                                    % Check both wheel constraint equation
                                    % during collision to put back the
                                    % both wheel constraint again
                                    if fw_h > 0 && fw_h < params.model.geom.wheel.r + 0.005 && x_IC(2) > 0 && x_IC(2) < params.model.geom.wheel.r + 0.005 
                                        disp('Put flat_ground constraint on again')
                                        params.sim.constraints = ['flat_ground'];
                                    end

                             end
                             
                        case ['bw_off']
                            disp("BW Collision")
                            disp(x_IC(1))
                             [A_unilateral,~] = constraint_derivatives(x_IC,params); 
                             A_col = A_unilateral(2,:); %add new constraint row to A matrix
                             restitution = 1 + params.model.dyn.wheel_res;
                             Minv_col = inv_mass_matrix(x_IC,params);
                             x_IC(6:10) = x_IC(6:10) - (Minv_col*A_col'*inv(A_col*Minv_col*A_col')*diag(restitution)*A_col*x_IC(6:10)')';
                            % Check back wheel constraint equation
                            % during collision to put back the back
                            % wheel constraint again
                            if x_IC(2) > 0 && x_IC(2) < params.model.geom.wheel.r + 0.005
                                disp('Put Backwheel constraint on again')
                                params.sim.constraints = ['flat_ground'];
                            end
                            
                    end
                    
                end

        end               
    end
end 


 %% Plotting and Animation 
 figure(1);
 
 xplot = xsim';
 disp(max(xsim(2,:)))
 
 switch params.sim.trick
     
     case 'Backflip'
         
         %Plot the y position of the bike and time
         subplot(3,1,1), plot(tsim,xplot(2,:),'r-')
         yline(0.3)
         xlabel('Time (s)')
         ylabel('Y position (m)') 
         set(gca,'FontSize',12)
         title('Y position vs Time ','FontSize',12)

         % plot the velocity of the back wheel (rad/s) and time
         subplot(3,1,2), plot(tsim,-1*xplot(9,:),'b-')
         xlabel('Time (s)')
         ylabel('Back wheel Velocity (rad/s)') 
         set(gca,'FontSize',12)
         title('Back wheel velocity vs time','FontSize',12)

         % plot the magnitude of the bike's velocity and time
         vel_mag = sqrt(xplot(6,:).^2 + xplot(7,:).^2);
         subplot(3,1,3), plot(tsim,vel_mag,'r:','LineWidth',2);
         xlabel('Time (s)')
         ylabel('Linear velocity of bike (m/s)')
         set(gca,'FontSize',12)
         title('Linear velocity of bike vs Time','FontSize',12)

        figure(2);
         %Plot the angular velocity of the bike and time
         subplot(2,1,1), plot(tsim, xplot(8,:),'b-')
         xlabel('Time (s)')
         ylabel('Angular velocity of bike (rad/s)') 
         set(gca,'FontSize',12)
         title('Angular velocity of bike vs Time','FontSize',12)

         % plot commanded tau values
         alltau(numel(tsim)) = 0;
         subplot(2,1,2), plot(tsim,-1*alltau,'r:','LineWidth',2);
         xlabel('Time (s)')
         ylabel('Torque (Nm)')
         set(gca,'FontSize',12)
         title('Commanded Torque to Back Wheel vs Time','FontSize',12)
         
     case 'Wheelie'
         %Plot center of mass angle vs time
         xplot3p = xplot(3,:);
         subplot(3,1,1), plot(tsim(1:end-50),xplot3p(1:end-50),'r:','LineWidth',2);
         xlabel('time')
         ylabel('angle')
         set(gca,'FontSize',11)
         title('Center of Mass Angle vs Time','FontSize',12)
         
         % plot commanded tau values
         alltau(numel(tsim)) = 0;
         subplot(3,1,2), plot(tsim(1:end-50),alltau(1:end-50),'r:','LineWidth',2);
         xlabel('time')
         ylabel('torque')
         set(gca,'FontSize',11)
         title('Commanded Torque to Back Wheel vs Time','FontSize',12)
         alldtheta_bw(numel(tsim)) = 0;
         
         %Plot wheel angular velocity and time
         subplot(3,1,3), plot(tsim(1:end-50),alldtheta_bw(1:end-50),'r:','LineWidth',2);
         xlabel('time')
         ylabel('angular velocity')
         set(gca,'FontSize',11)
         title('Angular Velocity of Back Wheel vs Time','FontSize',12)
         
         
 end
        
     

 
 pause(1); % helps prevent animation from showing up on the wrong figure
 
% Let's resample the simulator output so we can animate with evenly-spaced
% points in (time,state).
% 1) deal with possible duplicate times in tsim:
% (https://www.mathworks.com/matlabcentral/answers/321603-how-do-i-interpolate-1d-data-if-i-do-not-have-unique-values
tsim = cumsum(ones(size(tsim)))*eps + tsim;

%Get rid of duplicates to animate
anim_table = table(tsim,xsim);
[~,ia] = unique(anim_table.tsim);
anim_table_unique = anim_table(ia,:);

% 2) resample the duplicate-free time vector:
t_anim = 0:params.viz.dt:tsim(end);

% 3) resample the state-vs-time array:
x_anim = interp1(anim_table_unique.tsim, anim_table_unique.xsim, t_anim); %x_anim doesn't run in airborne
x_anim = x_anim'; % transpose so that xsim is 5xN (N = number of timesteps)
 
%Create animation from state array
animate_robot(x_anim(1:5,2:end),params,'trace_cart_com',false,...
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

                case ['bw_off']
                    A = A_all([1,3],:);
                    Adotqdot = [q_dot'*Hessian(:,:,1)*q_dot; % robot position x-constraint
                             q_dot'*Hessian(:,:,3)*q_dot]; % backwheel y-constraint
                    Fnow = (A*Minv*A')\(A*Minv*(Q - H) + Adotqdot);
                    dx(1:nq) = (eye(nq) - A'*((A*A')\A))*x(6:10);
                    dx(nq+1:2*nq) = Minv*(Q - H - A'*Fnow) - A'*((A*A')\A)*q_dot/params.sim.dt;
                    c_bw = x(2) - params.model.geom.wheel.r;

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
    
     
    % MATLAB Documentation
    % value, isterminal, and direction are vectors whose ith element corresponds to the ith event function:
    % value(i) is the value of the ith event function.
    % isterminal(i) = 1 if the integration is to terminate at a zero of this event function. Otherwise, it is 0.
    % direction(i) = 0 if all zeros are to be located (the default). A value of +1 locates only zeros where the event function is increasing, and -1 locates only zeros where the event function is decreasing.     
switch params.sim.trick 
    
    case 'Backflip'
 
         switch params.sim.constraints

             case ['flat_ground']
                 value = 0;%Event triggering is handled in the while loop of the discretized control
                 isterminal = 1; 
                 direction = 1; 

             case ['fw_ramp']
                 
                 value = x_bw_ramp; 
                 isterminal = 1;
                 direction = 0;

             case ['bw_ramp']
                 
                 
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
                  
             case ['bw_off']
                  value = c_bw;
                  isterminal = 1;
                  direction = -1;
         end
         
    case 'Wheelie'
      
      switch params.sim.constraints
          
          case ['flat_ground']
                 value = F_calc(3); % use the value corresponding to the front wheel constraint
                 isterminal = 1; 
                 direction = 1; 
          
          case ['fw_off']
                value = 0; %Event triggering is handled in the while loop of the discretized control
                isterminal = 1;
                direction = -1;
      end
 end
 
 end % end of robot_events


end
