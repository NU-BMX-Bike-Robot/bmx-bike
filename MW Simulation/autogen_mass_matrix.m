function M = autogen_mass_matrix(I_bike,I_mw,l_com,m_bike,m_mw)
%AUTOGEN_MASS_MATRIX
%    M = AUTOGEN_MASS_MATRIX(I_BIKE,I_MW,L_COM,M_BIKE,M_MW)

%    This function was generated by the Symbolic Math Toolbox version 8.2.
%    08-Jun-2020 00:03:56

t2 = l_com.^2;
M = reshape([I_bike+I_mw+m_bike.*t2+m_mw.*t2,I_mw,I_mw,I_mw],[2,2]);
