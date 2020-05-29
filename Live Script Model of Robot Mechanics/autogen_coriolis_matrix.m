function C = autogen_coriolis_matrix(dtheta_bw,dtheta_fw,dx_com,dy_com,m_bw,m_fw,theta_bw,theta_fw)
%AUTOGEN_CORIOLIS_MATRIX
%    C = AUTOGEN_CORIOLIS_MATRIX(DTHETA_BW,DTHETA_FW,DX_COM,DY_COM,M_BW,M_FW,THETA_BW,THETA_FW)

%    This function was generated by the Symbolic Math Toolbox version 8.5.
%    03-May-2020 21:27:53

t2 = theta_bw.*2.0;
t3 = theta_fw.*2.0;
t4 = cos(t2);
t5 = cos(t3);
t6 = sin(t2);
t7 = sin(t3);
t8 = dtheta_bw.*m_bw.*t4;
t9 = dtheta_fw.*m_fw.*t5;
t10 = dtheta_bw.*m_bw.*t6;
t11 = dtheta_fw.*m_fw.*t7;
t12 = t8+t9;
C = reshape([-t10-t11,t12,0.0,m_bw.*(dx_com.*t6-dy_com.*t4),m_fw.*(dx_com.*t7-dy_com.*t5),t12,t10+t11,0.0,m_bw.*(dx_com.*t4.*2.0+dy_com.*t6.*2.0).*(-1.0./2.0),m_fw.*(dx_com.*t5.*2.0+dy_com.*t7.*2.0).*(-1.0./2.0),0.0,0.0,0.0,0.0,0.0,-dx_com.*m_bw.*t6+dy_com.*m_bw.*t4,dx_com.*m_bw.*t4+dy_com.*m_bw.*t6,0.0,0.0,0.0,-dx_com.*m_fw.*t7+dy_com.*m_fw.*t5,dx_com.*m_fw.*t5+dy_com.*m_fw.*t7,0.0,0.0,0.0],[5,5]);
