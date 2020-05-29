function [tau_d,e_total,e_prev,status] = Controller(state,eint,prevError,tol)


status = "NA"; 

params = init_params;
if (params.sim.trick =="Backflip")
    Kp = 100;
    Ki = 5;
    Kd = 8; %1;
    
    set = -12.6; % speed calculated from winter quarter 
    error = set-state;
    tau_d = Kp * error + Ki * eint + Kd *(error-prevError)/params.sim.dt;
    e_total = eint+error;
    e_prev = error;
    status = "na";
    
elseif (params.sim.trick == "Wheelie")
    Kp = 150; %1000 %200 %0.5; %0.17 
    Ki = 0.05; %0.08 %0.08 %.0001;
    Kd = 200; %0.8 %0.008 %.00007;
    
    set = pi/2 - params.model.geom.bw_com.theta; 
        
    error = set-state;
    %display(error)
    
    e_total = eint+error;
    e_prev = error;
    
    tau_d = Kp * error + Ki * eint + Kd *(error-prevError)/params.sim.dt;
    
    if error < 0
        %display("neg")
        %error = state - set;
        status = "neg"; 
        %error = (90+error); 
    else
        status = "pos";
    end
    

    
end

eintmax = 3000;


%if (eint>eintmax)
%    eint = eintmax;
%elseif (eint<-eintmax)
%    eint = -eintmax;
%end

end
