function [A_all,H_com] = autogen_constraint_derivatives
%AUTOGEN_CONSTRAINT_DERIVATIVES
%    [A_ALL,H_COM] = AUTOGEN_CONSTRAINT_DERIVATIVES

%    This function was generated by the Symbolic Math Toolbox version 8.2.
%    24-May-2020 11:24:10

A_all = [0.0,0.0];
if nargout > 1
    H_com = reshape([0.0,0.0,0.0,0.0],[2,2]);
end