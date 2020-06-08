%% init_params.m
%
% Description:
%   Initializes the values of many parameters, such as parameters in the
%   system dynamics, parameters that relate to simulating the system
%   forward in time, and parametes that relate to visualization/animation.
%
% Inputs:
%   none

% Outputs:
%   params: a struct with many elements

function params = init_params
    % parameters that appear in the dynamics:
     params.model.dyn.com.m = 5;    % mass of the bike body
     params.model.dyn.mw.m = 0.647;    % mass of momentum wheel

     params.model.dyn.mw.I = 0.00127; % moment of inertia of MW
     params.model.dyn.com.I = 0.069; % moment of inertia of com
     
     params.model.dyn.com.l = 0.26; % Height of CoM above the ground

     params.model.dyn.g = 9.81;      % acceleration due to gravity
     params.model.dyn.tau_mw = -1;    % applied torque from the backwheel
     
    % parameters that help with visualizing the bike and MW:
    params.model.geom.body.h = 0.3; % height of the bike body
    params.model.geom.mw.r = 0.0635; % radius of wheels
    
    params.viz.colors.mw = [0.75 0.75 0.75];
    params.viz.colors.bike = [0.25 0.25 0.25];
    params.viz.colors.tracers.body = 'r';
    params.viz.colors.tracers.leg = 'g';
    params.viz.colors.tracers.wheels = 'b';
    params.viz.axis_lims = [-0.5,0.5,-0.5,0.5];
    params.viz.dt = 0.05;

    % parameters related to simulating (integrating) the dynamics forward
    % in time:
    params.sim.ICs.theta_bike = pi/2-5/180*pi;  % initial positions
    params.sim.ICs.theta_mw = 0;
    
    params.sim.ICs.dtheta_bike = 0;     % initial velocities
    params.sim.ICs.dtheta_mw = 0;
   
    params.sim.tfinal = 5;          % Length of the simulation
    params.sim.dt = 0.001;           % simulation timestep
    
    % variables related to the constraints
    params.sim.constraints.number = 1;  % total number of constraint equations
    
    % list of ground constraints True if wheel is on the ground; False otherwise
    params.sim.constraints = ['balancing']; % [back wheel, front wheel]
    
end
