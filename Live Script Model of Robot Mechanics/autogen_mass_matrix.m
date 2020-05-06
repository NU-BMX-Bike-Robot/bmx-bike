function M = autogen_mass_matrix(I_bw,I_com,I_fw,bw_com_distance,bw_com_init_angle,fw_com_distance,fw_com_init_angle,m_bw,m_com,m_fw,theta_com)
%AUTOGEN_MASS_MATRIX
%    M = AUTOGEN_MASS_MATRIX(I_BW,I_COM,I_FW,BW_COM_DISTANCE,BW_COM_INIT_ANGLE,FW_COM_DISTANCE,FW_COM_INIT_ANGLE,M_BW,M_COM,M_FW,THETA_COM)

%    This function was generated by the Symbolic Math Toolbox version 8.5.
%    05-May-2020 18:54:15

t2 = bw_com_init_angle+theta_com;
t3 = fw_com_init_angle+theta_com;
t6 = m_bw+m_com+m_fw;
t4 = cos(t2);
t5 = cos(t3);
t7 = sin(t2);
t8 = sin(t3);
t9 = bw_com_distance.*m_bw.*t4;
t10 = fw_com_distance.*m_fw.*t5;
t11 = bw_com_distance.*m_bw.*t7;
t12 = fw_com_distance.*m_fw.*t8;
t13 = -t9;
t14 = -t10;
t15 = -t12;
t16 = t11+t15;
t17 = t13+t14;
M = reshape([t6,0.0,t16,0.0,0.0,0.0,t6,t17,0.0,0.0,t16,t17,I_com+bw_com_distance.^2.*m_bw+fw_com_distance.^2.*m_fw,0.0,0.0,0.0,0.0,0.0,I_bw,0.0,0.0,0.0,0.0,0.0,I_fw],[5,5]);
