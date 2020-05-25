function Minv = autogen_inverse_mass_matrix(I_bike,I_mw,l_com,m_bike,m_mw,offset,theta_bike)
%AUTOGEN_INVERSE_MASS_MATRIX
%    MINV = AUTOGEN_INVERSE_MASS_MATRIX(I_BIKE,I_MW,L_COM,M_BIKE,M_MW,OFFSET,THETA_BIKE)

%    This function was generated by the Symbolic Math Toolbox version 8.2.
%    24-May-2020 11:24:13

t2 = l_com.^2;
t3 = sin(theta_bike);
t4 = t3.^2;
Minv = reshape([1.0./(I_bike+m_bike.*t2+m_mw.*offset.^2.*t4.*2.0+m_mw.*t2.*t4.*2.0+l_com.*m_mw.*offset.*t4.*4.0),0.0,0.0,1.0./I_mw],[2,2]);