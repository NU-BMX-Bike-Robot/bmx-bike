%% robot_dynamics.m
%
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

function [dx] = robot_dynamics_constraints(t,x,params)

% for convenience, define q_dot
dx = zeros(numel(x),1);
nq = numel(x)/2;    % assume that x = [q;q_dot];
q_dot = x(nq+1:2*nq);

% solve for control inputs at this instant
%tau_s = interp1(params.motor.spine.time,params.motor.spine.torque,t);
%tau_m = interp1(params.motor.body.time,params.motor.body.torque,t);
Q = [0;0;0;params.model.dyn.tau_bw;0];

% find the parts that don't depend on constraint forces
H = H_eom(x,params);
Minv = inv_mass_matrix(x,params);
[A_all,Hessian] = constraint_derivatives(x,params);

% build the constraints, forces, and solve for acceleration
A = A_all([1,2],:);
Adotqdot = [q_dot'*Hessian(:,:,1)*q_dot;
            q_dot'*Hessian(:,:,2)*q_dot];
            %q_dot'*Hessian(:,:,3)*q_dot];
Fnow = (A*Minv*A')\(A*Minv*(Q - H) + Adotqdot);
dx(1:nq) = (eye(nq) - A'*((A*A')\A))*x(6:10);
dx(nq+1:2*nq) = Minv*(Q - H - A'*Fnow);
%F = [Fnow;0;0];



% switch params.sim.constraints  
%     case ['false','false']     % both feet are off the ground
%         dx(1:nq) = q_dot;
%         dx(nq+1:2*nq) = Minv*(Q - H);
%         F = [0;0;0;0];
%     case ['true','false']      % left foot is on the ground and right is off
%         A = A_all([1,2],:);
%         Adotqdot = [q_dot'*Hessian(:,:,1)*q_dot;
%                     q_dot'*Hessian(:,:,2)*q_dot ];
%         Fnow = (A*Minv*A')\(A*Minv*(Q - H) + Adotqdot);
%         dx(1:nq) = (eye(nq) - A'*((A*A')\A))*x(6:10);
%         dx(nq+1:2*nq) = Minv*(Q - H - A'*Fnow);
%         F = [Fnow;0;0];
%     case ['false','true']      % right foot is on the ground and left is off
%         A = A_all([3,4],:);
%         Adotqdot = [q_dot'*Hessian(:,:,3)*q_dot;
%                     q_dot'*Hessian(:,:,4)*q_dot ];
%         Fnow = (A*Minv*A')\(A*Minv*(Q - H) + Adotqdot);
%         dx(1:nq) = (eye(nq) - A'*((A*A')\A))*x(6:10);
%         dx(nq+1:2*nq) = Minv*(Q - H - A'*Fnow);
%         F = [0;0;Fnow];
%     case ['true','true']      % both feet are on the ground
%         A = A_all([1,2,4],:);
%         Adotqdot = [q_dot'*Hessian(:,:,1)*q_dot;
%                     q_dot'*Hessian(:,:,2)*q_dot;
%                     q_dot'*Hessian(:,:,4)*q_dot ];
%         Fnow = (A*Minv*A')\(A*Minv*(Q - H) + Adotqdot);
%         dx(1:nq) = (eye(nq) - A'*((A*A')\A))*x(6:10);
%         dx(nq+1:2*nq) = Minv*(Q - H - A'*Fnow);
%         F = [Fnow(1);Fnow(2);0;Fnow(3)];
% end


end