% ftrimRoll.m
% function to be minimized to obtain steady turn flight conditions
% Usage
%     y = ftrimRoll(Xi,V,h,,rollRate,ihTrimRect,deltaCGb,aircraft)
%     where
%         Xi = [theta;alpha;beta;deltat;pitchControl,deltaa,deltar] : Vector with valriables to be calculates in stady flight roll conditions
%
%         with

%           theta : pich (rad)
%           alpha : angle of attack (rad)
%           beta = angle of slide slip (rad)
%           deltat : propulsion system control
%           pitchcontrol : pitch control.
%                   pitchControl = deltae if Cmdeltae~=0, otherwise pichtcontol = ih el ultimo solo para control de incidencia
%           deltar : ruder (rad)
%           deltaa : aileron (rad)
%           V : airspeed (m/s)
%           h : altitude (m)

%           rollRate: roll rate (rad/s)
%           ihTrimRect:  horizontal tail incidence control in steady rectilinear flight condition
%           deltaCGb : relative CG positions respect to nominal CG
%           aircraft : Data structure


function y = ftrimRoll(Xi,V,h,rollRate,ihTrimRect,deltaCGb,aircraft)

    global Vbdot % define global variable Vbdot

    %extract the components of Xi

    theta = Xi(1,1);
    alpha = Xi(2,1);
    beta = Xi (3);
    deltat = Xi(4,1);
    pitchControl = Xi(5,1);

    if aircraft.Cmdeltae ~= 0;
        ih = ihTrimRect; %Situciones donde la aeronave solo tiene control de incidencia, o tiene los dos controles, por tanto uso el control de incidencia de la cola horizontal
        deltae = pitchControl; %como el control de trim
    else
        ih = pitchControl;
        deltae = 0;
    end
    deltaa = Xi(6,1);
    deltar = Xi(7,1);

    % set stady tunr flight condition

        Vwe = [0;0;0];

        %aircraft position
        pe = [0;0;-h];

        %euler angles
        phi  = 0;
        psi = 0;
        Phi = [phi;theta;psi];

        %calculate reative velocity

        Vrelb = [V*cos(alpha)*cos(beta); V*sin(beta); V*sin(alpha)*cos(beta)];
        Vb = Vrelb;

        %derivate of Euler angles
        phidot = rollRate;
        thetadot = 0;
        psidot = 0;
        % calculate angular velocity
        Phidot = [phidot;thetadot;psidot];
        Omegab = Hinv(Phi) * Phidot;

        %state vector
        x = [pe;Phi;Vb;Omegab];

        %derivative of state vector
        Cbe = DCM(Phi);
        pedot = Cbe'*Vb;
        Vbdot = [0;0;0];
        Omegabdot = [0;0;0];

        %assemble  xdot
        xdot = [pedot; Phidot;Vbdot; Omegabdot];

        % time (don't care)
        t = 0;

        % contols vector
        deltaf = 0;
        delta = [deltat; deltaf;ih;deltae;deltaa;deltar];



y = norm(xdot - faircraft(t,x,delta,Vwe, deltaCGb, aircraft))^2+(pedot(3))^2;


end





