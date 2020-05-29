function tau = Motor(V,omega)

% V: voltage
% omega: motor speed 

% 4:1 gear ratio
speed = omega*4;

% motor resistance 
R = 0.005;

% back EMF constant
Ke = 1/2100;

% motor constant Kt
Kv = 2100;
Kt = (3/2)*(60/(2*pi))*(1/Kv);

backEMF = 0; %Ke*speed;

tau = Kt*(V-backEMF)/R;

end
