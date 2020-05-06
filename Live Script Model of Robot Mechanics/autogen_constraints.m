function C_all = autogen_constraints(bw_com_distance,bw_com_init_angle,fw_com_distance,fw_com_init_angle,r_bw,theta_bw,theta_com,x_com,y_com)
%AUTOGEN_CONSTRAINTS
%    C_ALL = AUTOGEN_CONSTRAINTS(BW_COM_DISTANCE,BW_COM_INIT_ANGLE,FW_COM_DISTANCE,FW_COM_INIT_ANGLE,R_BW,THETA_BW,THETA_COM,X_COM,Y_COM)

%    This function was generated by the Symbolic Math Toolbox version 8.5.
%    05-May-2020 18:54:14

t2 = -r_bw;
C_all = [x_com+t2.*theta_bw;t2+y_com-bw_com_distance.*sin(bw_com_init_angle+theta_com);t2+y_com-fw_com_distance.*sin(fw_com_init_angle+theta_com)];
