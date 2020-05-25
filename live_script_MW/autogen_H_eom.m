function H = autogen_H_eom(dtheta_bike,g,l_com,m_bike,m_mw,offset,theta_bike)
%AUTOGEN_H_EOM
%    H = AUTOGEN_H_EOM(DTHETA_BIKE,G,L_COM,M_BIKE,M_MW,OFFSET,THETA_BIKE)

%    This function was generated by the Symbolic Math Toolbox version 8.2.
%    24-May-2020 11:24:13

t2 = dtheta_bike.^2;
t3 = theta_bike.*2.0;
t4 = sin(t3);
H = [g.*l_com.*m_bike.*cos(theta_bike)+l_com.^2.*m_mw.*t2.*t4+m_mw.*offset.^2.*t2.*t4+l_com.*m_mw.*offset.*t2.*t4.*2.0;0.0];