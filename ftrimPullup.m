
% ftrimPullup.m
% function to be minimized to obtain steady pull up/pull over flight conditions
% Usage 
%       y = ftrimPullup(Xi,V,h,pitchRate,ihTrimRect,deltaCGb,aircraft)

%     where 
%         Xi = [alpha;deltat;pitchControl] : Vector of state to calaculates in steady rectilinear flight 
% 
%         with 
%             alpha : angle of attack (rad)
%             deltat : propulsion system control
%             pitchcontrol : pitch control. 
%                  pitchControl = deltae if Cmdeltae~=0, otherwise pichtcontol = ih el ultimo solo para control de incidencia 
%             
%           V : airspeed (m/s)
%           h : altitude (m)
%           pitchRate = pitch rade (rad/s)
%           ihTrimRect:  horizontal tail incidence control in steady rectilinear flight condition 
%           deltaCGb : relative CG positions respect to nominal CG 
%           aircraft : Data structure 


function y = ftrimPullup(Xi,V,h,pitchRate,ihTrimRect,deltaCGb,aircraft)

    global Vbdot % define global variable Vbdot

    %extract the components of Xi
    alpha = Xi(1,1);
    deltat = Xi(2,1);
    pitchControl = Xi(3,1); 
     if aircraft.Cmdeltae ~= 0;
        ih = ihTrimRect; %Situciones donde la aeronave solo tiene control de incidencia, o tiene los dos controles, por tanto uso el control de incidencia de la cola horizontal
        deltae = pitchControl; %como el control de trim
     else
        ih = pitchControl;
        deltae = 0;
     end
    


    if aircraft.Cmih ~= 0
        ih = pitchControl; %Situciones donde la aeronave solo tiene control de incidencia, o tiene los dos controles, por tanto uso el control de incidencia de la cola horizontal
        deltae = 0; %como el control de trim
    else
        ih = 0;
        deltae = pitchControl;
    end

    % set stady pull up / pull over  flight condition
        
        Vwe = [0;0;0];
    
        %aircraft position
        pe = [0;0;-h];
    
        %euler angles
        phi = 0;
        theta = alpha;
        psi = 0;
        Phi = [phi;theta;psi];

        %calculate reative velocity
        beta = 0;
        Vrelb = [V*cos(alpha)*cos(beta); V*sin(beta); V*sin(alpha)*cos(beta)];
        Vb = Vrelb;
        
        %derivate of Euler angles 
        phidot = 0;
        thetadot = pitchRate;
        psidot = 0;
        % calculate angular velocity 
        Phidot = [phidot; thetadot;psidot];
        Omegab = Hinv(Phi) * Phidot;

        %state vector
        x = [pe;Phi;Vb;Omegab];

        %derivative of state vector 
        Cbe = DCM(Phi);
        pedot = Cbe'*Vb;
        Vbdot = [0;0;0];
        Omegabdot = [0;0;0];

        %assemble  xdot 
        xdot = [pedot;Phidot;Vbdot;Omegabdot];

        % time (don't care)
        t = 0;

        % contols vector 
        deltaf = 0;
        deltaa = 0;
        deltar = 0;
        delta = [deltat; deltaf;ih;deltae;deltaa;deltar];
       
 

        y = norm(xdot - faircraft(t,x,delta,Vwe, deltaCGb, aircraft))^2;

end 
    



                