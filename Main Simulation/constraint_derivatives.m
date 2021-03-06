%% constraint_derivatives.m
%
% Description:
%   Wrapper function for autogen_constraint_derivatives.m
%   Computes the constraint jacobian and hessians for the jumping robot.
%
% Inputs:
%   x: the state vector, x = [q; q_dot];
%   params: a struct with many elements, generated by calling init_params.m
%
% Outputs:
%   A_all: a 3x3 jacobian of constraint equation derivatives.  If only a
%   subset of constraints are active, then only those rows of A_all will be
%   used to compute the A matrix for that situation.
%
%   H_c1, H_c2, H_c3:  the hessian matrices; one for each constraint.  Note
%   that H_c3 is the null matrix, but we keep it for clean, robust code.

function [A_all,Hessian] = constraint_derivatives(x,params)

theta_com = x(3);

r_bw = params.model.geom.wheel.r;
r_fw = params.model.geom.wheel.r;
bw_fw_distance = params.model.geom.bw_fw.l;
x_ramp = params.model.geom.ramp.center.x;
y_ramp = params.model.geom.ramp.center.y;
r_ramp = params.model.geom.ramp.r;

x_bf = x(1);
y_bf = x(2);


[A_all,H_cbw_x,H_cbw_y,H_cfw_y, H_cfw_bw,H_bw_ramp,H_fw_ramp] = autogen_constraint_derivatives(bw_fw_distance,r_bw,r_fw,theta_com,x_bf,x_ramp,y_bf,y_ramp);
%AUTOGEN_CONSTRAINT_DERIVATIVES);
Hessian = cat(3,H_cbw_x,H_cbw_y,H_cfw_y,H_cfw_bw,H_bw_ramp,H_fw_ramp);

end