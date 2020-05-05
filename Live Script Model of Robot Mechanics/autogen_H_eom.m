function H = autogen_H_eom(bw_com_distance,bw_com_init_angle,dtheta_bw,dtheta_fw,dtheta_com,dx_com,dy_com,fw_com_distance,fw_com_init_angle,g,m_bw,m_com,m_fw,theta_bw,theta_fw,theta_com)
%AUTOGEN_H_EOM
%    H = AUTOGEN_H_EOM(BW_COM_DISTANCE,BW_COM_INIT_ANGLE,DTHETA_BW,DTHETA_FW,DTHETA_COM,DX_COM,DY_COM,FW_COM_DISTANCE,FW_COM_INIT_ANGLE,G,M_BW,M_COM,M_FW,THETA_BW,THETA_FW,THETA_COM)

%    This function was generated by the Symbolic Math Toolbox version 8.5.
%    05-May-2020 15:57:57

t2 = bw_com_init_angle+theta_com;
t3 = fw_com_init_angle+theta_com;
t4 = bw_com_init_angle.*2.0;
t5 = bw_com_distance.^2;
t6 = dtheta_com.^2;
t7 = dx_com.^2;
t8 = dy_com.^2;
t9 = fw_com_init_angle.*2.0;
t10 = fw_com_distance.^2;
t11 = theta_bw.*2.0;
t12 = theta_fw.*2.0;
t13 = theta_com.*2.0;
t14 = cos(t11);
t15 = cos(t12);
t16 = sin(t11);
t17 = sin(t12);
t18 = cos(t2);
t19 = cos(t3);
t20 = -t11;
t21 = t3+t12;
t22 = t2+t20;
t23 = cos(t21);
t24 = sin(t21);
t27 = t3+t21;
t25 = cos(t22);
t26 = sin(t22);
t28 = sin(t27);
t29 = t2+t22;
t30 = sin(t29);
H = [-dtheta_bw.*dx_com.*m_bw.*t16-dtheta_fw.*dx_com.*m_fw.*t17+dtheta_bw.*dy_com.*m_bw.*t14+dtheta_fw.*dy_com.*m_fw.*t15+(bw_com_distance.*m_bw.*t6.*t18)./2.0+(bw_com_distance.*m_bw.*t6.*t25)./2.0-(fw_com_distance.*m_fw.*t6.*t19)./2.0-(fw_com_distance.*m_fw.*t6.*t23)./2.0-bw_com_distance.*dtheta_bw.*dtheta_com.*m_bw.*t25-dtheta_fw.*dtheta_com.*fw_com_distance.*m_fw.*t23;g.*m_bw+g.*m_com+g.*m_fw+dtheta_bw.*dx_com.*m_bw.*t14+dtheta_fw.*dx_com.*m_fw.*t15+dtheta_bw.*dy_com.*m_bw.*t16+dtheta_fw.*dy_com.*m_fw.*t17-(bw_com_distance.*m_bw.*t6.*t26)./2.0-(fw_com_distance.*m_fw.*t6.*t24)./2.0+(bw_com_distance.*m_bw.*t6.*sin(t2))./2.0+(fw_com_distance.*m_fw.*t6.*sin(t3))./2.0+bw_com_distance.*dtheta_bw.*dtheta_com.*m_bw.*t26-dtheta_fw.*dtheta_com.*fw_com_distance.*m_fw.*t24;-bw_com_distance.*g.*m_bw.*t18-fw_com_distance.*g.*m_fw.*t19+(m_bw.*t5.*t6.*t30)./2.0+(m_fw.*t6.*t10.*t28)./2.0-bw_com_distance.*dtheta_bw.*dx_com.*m_bw.*t25+bw_com_distance.*dtheta_bw.*dy_com.*m_bw.*t26-dtheta_fw.*dx_com.*fw_com_distance.*m_fw.*t23-dtheta_fw.*dy_com.*fw_com_distance.*m_fw.*t24-dtheta_bw.*dtheta_com.*m_bw.*t5.*t30+dtheta_fw.*dtheta_com.*m_fw.*t10.*t28;(m_bw.*(t7.*t16-t8.*t16-dx_com.*dy_com.*t14.*2.0+t5.*t6.*t30+bw_com_distance.*dtheta_com.*dx_com.*t25.*2.0-bw_com_distance.*dtheta_com.*dy_com.*t26.*2.0))./2.0;(m_fw.*(t7.*t17-t8.*t17-dx_com.*dy_com.*t15.*2.0-t6.*t10.*t28+dtheta_com.*dx_com.*fw_com_distance.*t23.*2.0+dtheta_com.*dy_com.*fw_com_distance.*t24.*2.0))./2.0];
