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
tol = 0.001; 

% create a place for constraint forces populated in
% robot_dynamic_constraints function
F_calc = [];
tsim = [];
xsim = [];
xfin = [];

x_fw_ramp = 100; %x_position when the frontwheel hits the ramp
x_bw_ramp = 100; %x_position when the backwheel hits the ramp
fw_ang_compare = 100;  %y_position when the frontwheel leaves the ramp
bw_ang_compare = 100;  %y_position when the backwheel leaves the ramp

% Set integration options - mainly events
options = odeset('Events',@robot_events);

while params.sim.tfinal - t_curr > params.sim.dt
        
    tspan_passive = t_curr:params.sim.dt:params.sim.tfinal;
    
    [tseg, xseg, ~, ~, ~] = ode45(@robot_dynamics_constraints, tspan_passive, x_IC', options);
    
    % extract info from the integration
    tsim = [tsim;tseg]; % build up the time vector after each event
    xsim = [xsim;xseg]; % build up the calculated state after each event
    
    xfin = xseg(end,:);
    
    t_curr = tsim(end); % set the current time to where the integration stopped
    x_IC = xsim(end,:); % set the initial condition to where the integration stopped
    
    % if the simulation ended early, specify the new set of constraints
        
        switch params.sim.trick
            
            case 'Backflip'
                
                if  params.sim.tfinal - tseg(end) > params.sim.dt %&& x_IC(1)>2.7 %FIXME this second condition should be necessary!
                    %but backflip constraints only work when this is true
                    %(x(1) jumps drastically while being integrated) find a
                    %way to correct this or add a better condition
                
                    switch params.sim.constraints

                        case ['flat_ground'] %both wheels are on the ground
                            disp("FW is on the ramp!")
                            params.sim.constraints = ['fw_ramp'];
                        case ['fw_ramp']     %only the front wheel is on the ramp
                            disp("BW is on the ramp!")
                            params.sim.constraints = ['bw_ramp'];
                        case ['bw_ramp']     %both wheels are on the ramp
                            disp("FW has left the ramp")
                            params.sim.constraints = ['fw_airborne'];
                        case ['fw_airborne']     %frontwheels leaves the ramp
                            disp("BW has left the ramp")
                            params.sim.constraints = ['bw_airborne'];
                    end
                    
                end
                
            case 'Wheelie'
                
                if  params.sim.tfinal - tseg(end) > params.sim.dt
                
                    switch params.sim.constraints

                         case ['flat_ground'] % both wheels are on the ground
                             disp("Changed Constraint!")
                             params.sim.constraints = ['fw_off']; % the front wheel is now off the ground    end
                    end
                          
                end
        end         
end

% transpose xsim_passive so that it is 5xN (N = number of timesteps):
 
 figure;
 
 xplot = xsim';
  
 % plot the x and y position of the back wheel
 subplot(2,1,1), plot(tsim,xplot(1,:),'b-',...
                      tsim,xplot(2,:),'r-','LineWidth',2);
 lgd1 = legend({'x position back wheel','y position back wheel'},'Location','southwest');
 lgd1.FontSize = 10;
 xlabel('time')
 ylabel('position') 
 
 % plot the angle of the COM and the back wheel
 %subplot(2,1,2), plot(tsim,xplot(3,:),'b:',...
 %                     tsim,xplot(4,:),'r:','LineWidth',2);
 subplot(2,1,2), plot(tsim,xplot(3,:),'r:','LineWidth',2);
 lgd2 = legend({'angle COM','angle back wheel'},'Location','southwest');
 lgd2.FontSize = 10; 
 xlabel('time')
 ylabel('angle')
 pause(1); % helps prevent animation from showing up on the wrong figure
 
% Let's resample the simulator output so we can animate with evenly-spaced
% points in (time,state).
% 1) deal with possible duplicate times in tsim:
% (https://www.mathworks.com/matlabcentral/answers/321603-how-do-i-interpolate-1d-data-if-i-do-not-have-unique-values
tsim = cumsum(ones(size(tsim)))*eps + tsim;

% 2) resample the duplicate-free time vector:
t_anim = 0:params.viz.dt:tsim(end);

% 3) resample the state-vs-time array:
% x_anim = interp1(tsim, xsim, t_anim); %x_anim doesn't run in airborne
x_anim = xsim'; % transpose so that xsim is 5xN (N = number of timesteps)
 
 animate_robot(x_anim(1:5,:),params,'trace_cart_com',false,...
     'trace_pend_com',false,'trace_pend_tip',false,'video',true);
 
 fprintf('Done passive simulation.\n');



function [dx] = robot_dynamics_constraints(t,x)
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
    
    theta_bw = x(4);    % Angular Position of back wheel
    dtheta_bw = x(9);   % Angular Velocity of back wheel
    theta_COM = x(3);   % Angular Position of COM
    omega_est = (theta_bw-prev_theta)/params.sim.dt;    % Use encoder count to approximate angular velocity
    prev_theta = theta_bw;      % Store last used angular position

    %tau = params.model.dyn.tau_bw * 0.05;
    %{
    if theta_COM>2*pi
        theta_COM=theta_COM-(2*pi);
    end
    
    if theta_COM<-2*pi
        theta_COM=theta_COM+(2*pi);
    end
    %}
    %tau = -3.5;

    %if t < 1
    %    tau = params.model.dyn.tau_bw;
    %end
    
    %display(tau);

    trick = params.sim.trick;
    
    if (trick == "Backflip")% && params.sim.constraints~="bw_airborne")
        %Speed Control % 
        [tau_d,tmpeint,tmperror,status] = Controller(dtheta_bw,eint,prevError,tol);
        prevError = tmperror; 
        eint = tmpeint; 

        tau = tau_d; %-3.45;
        display(status);
        %tau = Motor(tau_d,dtheta_bw);
        %display(dtheta_bw)
        display(tau);
        % Limit torque to feasible values
        %if tau>2.5
        %    tau = 2.5;
        %elseif tau<-2.5
        %    tau = -2.5;
        %end
    elseif (trick == "Wheelie")
        
        % Wheelie %
        %display(theta_COM)
        if (t<0.5)
            tau = -0.5;
        end
        
        if(t>=0.5&& t<0.7)
            tau = -2.5; 
        end

        
        if (t>=0.7)
            %display(theta_COM)
            [tau_d,eint,prevError,status] = Controller(theta_COM,eint,prevError,tol);
            tau = -Motor(tau_d,dtheta_bw); 
            
            %tau = 0; 
            display(tau);
            %{
            if status =="neg"
                tau = Motor(tau_d,dtheta_bw); 
            else 
                tau = -Motor(tau_d,dtheta_bw); 
            end
            %}
        end
        
        

        %{
        if (t>=1.4)
            [tau_d,eint,prevError,status] = Controller(theta_COM,eint,prevError,tol);
            tau = -Motor(tau_d,dtheta_bw); 
            
            %tau = 0; %2.5;
            %display(tau)
        end

        
        if (status=="balanced" && abs(dtheta_bw)>0.5)
            tau = -tau;

            %display(tau)
            %display(theta_COM)
            display("balanced")
        elseif (status == "balanced" && abs(dtheta_bw)<0.5)
            tau = 0;
        end
        %}
        %display(tau)
        % Limit torque to feasible values
        if tau> 2.5
            tau = 2.5;
        elseif tau<-2.5
            tau = -2.5;
        end

        
        
        
        %{
        if tau_d < 2.5
            tau = 0;
        elseif tau_d < 2.7
            tau = -3;
        else 
            [tau_d,eint,prevError] = Controller(theta_COM,eint,prevError);
            tau = Motor(tau_d,dtheta_bw);
        end
        % Limit torque to feasible values
        if tau>2.5
            tau = 2.5;
        elseif tau<-3
            tau = 3;
        end
        
        %}
    end
    
    
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
                    dx(nq+1:2*nq) = Minv*(Q - H - A'*Fnow);
                    x_fw_ramp = (x(1) + params.model.geom.bw_fw.l) - params.model.geom.ramp.center.x;

                case ['fw_ramp'] % front wheel is on the ramp
                    A = A_all([1,2,6],:);
                    Adotqdot = [q_dot'*Hessian(:,:,1)*q_dot;  % robot position x-constraint
                                q_dot'*Hessian(:,:,2)*q_dot;  % backwheel flat ground constraint
                                q_dot'*Hessian(:,:,6)*q_dot]; % frontwheel ramp constraint

                    Fnow = (A*Minv*A')\(A*Minv*(Q - H) + Adotqdot);
                    dx(1:nq) = (eye(nq) - A'*((A*A')\A))*x(6:10);
                    dx(nq+1:2*nq) = Minv*(Q - H - A'*Fnow);
                    x_bw_ramp = x(1) - params.model.geom.ramp.center.x;

                case ['bw_ramp'] % both wheels on the ramp
                    A = A_all([1,5,6],:);
                    Adotqdot = [q_dot'*Hessian(:,:,1)*q_dot;  % robot position x-constraint
                                q_dot'*Hessian(:,:,5)*q_dot;  % backwheel ramp constraint
                                q_dot'*Hessian(:,:,6)*q_dot]; % frontwheel ramp constraint

                    Fnow = (A*Minv*A')\(A*Minv*(Q - H) + Adotqdot);
                    dx(1:nq) = (eye(nq) - A'*((A*A')\A))*x(6:10);
                    dx(nq+1:2*nq) = Minv*(Q - H - A'*Fnow);

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
                    dx(nq+1:2*nq) = Minv*(Q - H - A'*Fnow);

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

                    mag_BA = sqrt(vec_BA(1)^2 + vec_BA(2)^2)
                    mag_BC = sqrt(vec_BC(1)^2 + vec_BC(2)^2)

                    dot_vecs = dot(vec_BA, vec_BC);
                    mags = mag_BA*mag_BC;

                    ang_ramp_bw = acos(dot_vecs / mags);

                    bw_ang_compare = ang_ramp_bw - params.model.geom.ramp.theta;

                case ['bw_airborne'] % both wheels leaves the ramp
                    dx(1:nq) = eye(nq)*x(6:10);
                    dx(nq+1:2*nq) = Minv*(Q - H);

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
                    dx(nq+1:2*nq) = Minv*(Q - H - A'*Fnow);
                    F_calc = Fnow;


                case ['fw_off'] % only the back wheel is on the ground
                     A = A_all([1,2],:);
                     Adotqdot = [q_dot'*Hessian(:,:,1)*q_dot; % robot position x-constraint
                                 q_dot'*Hessian(:,:,2)*q_dot]; % backwheel y-constraint
                     Fnow = (A*Minv*A')\(A*Minv*(Q - H) + Adotqdot);
                     dx(1:nq) = (eye(nq) - A'*((A*A')\A))*x(6:10);
                     dx(nq+1:2*nq) = Minv*(Q - H - A'*Fnow);
                     F_calc = [Fnow;0;0];

            end

    end
end
 
% Event handling Function
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

             case ['fw_ramp']
                 
                 value = x_bw_ramp;
                 isterminal = 1;
                 direction = 0;

             case ['bw_ramp']
                 
                 disp(fw_ang_compare);
                 
                 value = fw_ang_compare;
                 isterminal = 1;
                 direction = 0;

             case ['fw_airborne']
                 value = bw_ang_compare;
                 isterminal = 1;
                 direction = 0;

             case ['bw_airborne']
                 value = 1;
                 isterminal = 1;
                 direction = 0;
         end
         
    case 'Wheelie'
      
      switch params.sim.constraints
          
          case ['flat_ground']
                 value = F_calc(3); % use the value corresponding to the front wheel constraint
                 isterminal = 1; % tell ode45 to terminate if the event has occured
                 direction = 1; % tell ode45 to look for a positive constraint force as the event
          
          case ['fw_off']
              % Do not try and terminate once the wheel is off the ground.
              % Eventually will add in the collision detection here get back to
              % the ground.
                value = 1;
                isterminal = 0;
                direction = 0;
      end
 end
 
 end % end of robot_events

%%
% %% Control the unstable equilibrium with LQR
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
