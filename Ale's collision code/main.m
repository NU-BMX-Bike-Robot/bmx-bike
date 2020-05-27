3%% main.m
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

% create a place for constraint forces populated in
% robot_dynamic_constraints function
F_calc = [];
tsim = [];
xsim = [];

x_fw_ramp = 100; %x_position when the frontwheel hits the ramp
x_bw_ramp = 100; %x_position when the backwheel hits the ramp
y_fw_top = -100;  %y_position relative to ramp when the frontwheel leaves the ramp
y_bw_top = -100;  %y_position relative to ramp when the backwheel leaves the ramp

c_fw = 0;
c_bw = 0;
% Set integration options - mainly events
options = odeset('Events',@robot_events);%'RelTol',5e-2); %,'AbsTol',1e-3);

while params.sim.tfinal - t_curr > params.sim.dt
        
    tspan_passive = t_curr:params.sim.dt:params.sim.tfinal;
    
    [tseg, xseg, te, ye, ie] = ode45(@robot_dynamics_constraints, tspan_passive, x_IC', options);
    
    % extract info from the integration
    tsim = [tsim;tseg]; % build up the time vector after each event
    xsim = [xsim;xseg]; % build up the calculated state after each event
    
    t_curr = tsim(end); % set the current time to where the integration stopped
    x_IC = xsim(end,:); % set the initial condition to where the integration stopped
    
    % if the simulation ended early, specify the new set of constraints
        
        switch params.sim.trick
            
            case 'Backflip'
                
                if  params.sim.tfinal - tseg(end) > params.sim.dt %&& x_IC(1)>2.7 %FIXME this second condition shouldn't be necessary!
                    %but backflip constraints only work when this is true
                    %(x(1) jumps drastically while being integrated) find a
                    %way to correct this or add a better condition
                
                    switch params.sim.constraints
        
                        case ['flat_ground'] %both wheels are on the ground
                            %if(x_IC(1) - params.model.geom.ramp.center.x +params.model.geom.bw_fw.l > 0)
                            disp("Changed to fw_ramp constraint!")
                            %disp("fw_x_pos = %.2f",ye)
                            params.sim.constraints = ['fw_ramp'];
                            %end
                        case ['fw_ramp']     %only the front wheel is on the ramp
                            disp("Changed to bw_ramp constraint!")
                            params.sim.constraints = ['bw_ramp'];
                        case ['bw_ramp']     %both wheels are on the ramp
                            disp("Changed to fw_airborne constraint!")
                            params.sim.constraints = ['fw_airborne'];
                        case ['fw_airborne']     %frontwheels leaves the ramp
                            disp("Changed to bw_airborne constraint!")
                            params.sim.constraints = ['bw_airborne'];
                         case ['bw_airborne']
                             fw_h = x_IC(2) + params.model.geom.bw_fw.l*cos(x_IC(3));
                             if(x_IC(2) < params.model.geom.wheel.r + 0.01)
                                  disp("Collision")
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
                                if x_IC(2) > 0 && x_IC(2) < params.model.dyn.collision_threshold
                                     disp('Put frontwheel constraint on again')
                                     params.sim.constraints = ['flat_ground'];
                                end
                             
                             elseif(fw_h < params.model.geom.wheel.r + 0.01)
                                     disp("Collision")
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
                           
                             end
                    end
                    
                end
                
            case 'Wheelie'
                
                if  params.sim.tfinal - tseg(end) > params.sim.dt
                
                    switch params.sim.constraints

                         case ['flat_ground'] % both wheels are on the ground
                             disp("Changed Constraint!")
                             params.sim.constraints = ['fw_off']; % the front wheel is now off the ground 
                             
                         case ['fw_off'] % both wheels are on the ground
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

% transpose xsim_passive so that it is 5xN (N = number of timesteps):
 
 figure;
 
 xplot = xsim';
  
 % plot the x and y position of the back wheel
 subplot(2,1,1), plot(tsim,xplot(1,:),'b-',...
                      tsim,xplot(2,:),'r-','LineWidth',2);
                  
 % plot the angle of the COM and the back wheel
 subplot(2,1,2), plot(tsim,xplot(3,:),'b:',...
                      tsim,xplot(4,:),'r:','LineWidth',2);

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

tau = params.model.dyn.tau_bw*0.05;


switch params.sim.trick
    
    case 'Wheelie'
        if t > 1 && t < 1.15
            tau = params.model.dyn.tau_bw;
        elseif t < 1.3 && t > 1.15
            tau = 0;
        elseif t > 2
            tau = params.model.dyn.tau_bw*0.1;
        end
    case 'Backflip' 
        
        if t > params.model.ramp_up_start && t < params.model.ramp_up_end
            tau = params.model.dyn.tau_bw;
        else
            tau = 0;
        end
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
                dx(nq+1:2*nq) = Minv*(Q - H - A'*Fnow); %- A'*((A*A')\A)*q_dot/params.sim.dt;
                x_fw_ramp = x(1) - params.model.geom.ramp.center.x + params.model.geom.bw_fw.l; 

            case ['fw_ramp'] % front wheel is on the ramp
                A = A_all([1,2,6],:);
                Adotqdot = [q_dot'*Hessian(:,:,1)*q_dot; 
                            q_dot'*Hessian(:,:,2)*q_dot;  % backwheel y-constraint
                            q_dot'*Hessian(:,:,6)*q_dot];  % frontwheel is on the ramp

                Fnow = (A*Minv*A')\(A*Minv*(Q - H) + Adotqdot);
                dx(1:nq) = (eye(nq) - A'*((A*A')\A))*x(6:10);
                dx(nq+1:2*nq) = Minv*(Q - H - A'*Fnow); %- A'*((A*A')\A)*q_dot/params.sim.dt;
                x_bw_ramp = x(1) - params.model.geom.ramp.center.x;

            case ['bw_ramp'] % both wheels on the ramp
                A = A_all([1,5,6],:);
                Adotqdot = [q_dot'*Hessian(:,:,1)*q_dot; 
                            q_dot'*Hessian(:,:,5)*q_dot;  % backwheel y-constraint
                            q_dot'*Hessian(:,:,6)*q_dot];  % frontwheel is on the ramp

                Fnow = (A*Minv*A')\(A*Minv*(Q - H) + Adotqdot);
                dx(1:nq) = (eye(nq) - A'*((A*A')\A))*x(6:10);
                dx(nq+1:2*nq) = Minv*(Q - H - A'*Fnow); %- A'*((A*A')\A)*q_dot/params.sim.dt;

                %vertical height between fw and bw on the ramp
                %height_fw_bw = params.model.geom.body.w*cos(0.5*acos(1-(params.model.geom.body.w^2/(2*params.model.geom.ramp.r^2))));
                height_fw_bw = params.model.geom.bw_fw.l*sin(x(3));
                y_fw_top = x(2) - (params.model.geom.ramp.h - height_fw_bw);%(params.model.geom.ramp.y - height_fw_bw);

            case ['fw_airborne'] % front wheel leves the ramp
                A = A_all([1,5],:);
                Adotqdot = [q_dot'*Hessian(:,:,1)*q_dot; 
                            q_dot'*Hessian(:,:,5)*q_dot];  % only frontwheel leaves the ramp

                Fnow = (A*Minv*A')\(A*Minv*(Q - H) + Adotqdot);
                dx(1:nq) = (eye(nq) - A'*((A*A')\A))*x(6:10);
                dx(nq+1:2*nq) = Minv*(Q - H - A'*Fnow); %- A'*((A*A')\A)*q_dot/params.sim.dt;
                y_bw_top = x(2) - params.model.geom.ramp.h;

            case ['bw_airborne'] % both wheels leaves the ramp
                dx(1:nq) = eye(nq)*x(6:10);
                dx(nq+1:2*nq) = Minv*(Q - H); 
                c_bw = x(2) - params.model.geom.wheel.r;
                c_fw = x(2) + params.model.geom.bw_fw.l*sin(x(3)) - params.model.geom.wheel.r;
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
                 direction = 0; % tell ode45 to look for a positive constraint force as the event
                 disp(x_fw_ramp);

             case ['fw_ramp']
                 value = x_bw_ramp;
                 isterminal = 1;
                 direction = 1;
                 disp(x_bw_ramp);

             case ['bw_ramp']
                 value = y_fw_top;
                 isterminal = 1;
                 direction = 1;

             case ['fw_airborne']
                 value = y_bw_top;
                 isterminal = 1;
                 direction = 1;

             case ['bw_airborne']
                 value = [c_bw,c_fw];
                 isterminal = [1,1];
                 direction = [-1,-1];
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
                value = c_fw;
                isterminal = 1;
                direction = -1; %check when wheel height becomes negative to signal collision
      end
 end
 
 end % end of robot_events


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