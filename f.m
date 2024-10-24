% f.m
%Funtion to define the mathematical model for a fix wing aircraft
% Such that f(xdot,x,u) = 0

% y = f(xdot,x,u,h,deltaCGb,aircraft)
% Where
%
%   xdot =  [Vdot;betadot;alphadot;phidot;thetadod;psidot;psdot;qdot;rsdot] = [Vdot;betadot;alphadot;Phidot;Omegasdot] : derivate state vector
%      x = [V;beta;alpha;phi;tetha;psi;ps;q;rs]
%   with
%           V = airspeed (m/s)
%            beta = angle of slip (rad)
%            alpha = angle of attack (rad)
%            Phi = [phi;tetha;psi] euler angles /rad)
% %            Omegas = [ps;q;rs] aircraf angular velocity respect to eart expressions
%           u = [delta;ih;deltae;deltaa;deltar] aircraft controls vector
%

%       with
%           deltat : propulsion system control (0<=deltat<=1)
%           deltaf : flaps (rad)
%           ih : horizontal control incidence control (rad)
%           deltae : elevator (rad)
%           deltaa : aileron (rad)
%           deltar : ruder (rad)

%  h: altitde
%   deltaCGb : aircraft CG relative position respect to nominal CG
%   aircraft : aircraft data structure
%   y : output funtion

%

function y = f(xdot,x,u,h,deltaCGb,aircraft)



    % extract components of xdot, x and u
%   Xdot
    Vdot = xdot(1,1);
    betadot = xdot(2,1);
    alphadot= xdot(3,1);
    Phidot = xdot(4:6,1);
    Omegasdot = xdot(7:9,1);

    % x
    V = x(1,1);
    beta = x(2,1);
    alpha =x(3,1);
    Phi = x(4:6,1);
    Omegas =  x(7:9,1);

    % u
    deltat = u(1,1);
    deltaaero = [0;u(2:5,1)];


    %evaluate rotational kinematics
    Cbs = Cbsmatrix(alpha);
    Omegab = Cbs * Omegas;
    y2 = -Phidot + H(Phi)*Omegab;      %segunda ecuacion

    %calculate forces and moments acting on the aircraft

    %Weight

        Ge = [0;0;aircraft.g];
        Cbe = DCM(Phi);
        Gb = Cbe*Ge;
        Wb = aircraft.m*Gb;
        Cbw = Cbwmatrix(alpha,beta);
        Ww =  Cbw'*Wb;


    % calculate atmosphere parameters
        [rho,P,T,a] = atmosphere(h);

    %calculate dynamic pressure and MACH number
        qbar = rho*V^2/2;
        M = V/a;

    % calculate aerodynamics forces and moments
        [Fab,Mab] = aerodynamics(V,alpha,beta,alphadot,Omegab,deltaaero,qbar,M,deltaCGb,aircraft);

    % calculate aerodyamics forces and moments in the frame relative wind
    % and moemnts in the stability frame
        Faw =Cbw'*Fab;
        Mas =Cbs'*Mab;

    % calculate propulsion forces and moments
        [Ftb,Mtb] = propulsion(deltat,deltaCGb,aircraft);

    % calculate propulsion force express in relative wind frame and
    % propulsion moment expressed in stability frame
        Ftw =Cbw'*Ftb;
        Mts =Cbs'*Mtb;

    % Calculate net force expressed in wind frame
        Fnetw = Ww + Faw + Ftw;


    % calculate net moment expresed in sstability frame
        Mnets = Mas + Mts;

    % calculate other terms
        Vrelwdot = [Vdot;0;0];
        Vrelw = [V;0;0];
        Omegaw_b_w = [-sin(beta)*alphadot; -cos(beta)*alphadot; betadot];
        Omegaw = Cbw'*Omegab;
        Omegas_b_s = [0;-alphadot;0];
        Is = Cbs'*aircraft.Ib*Cbs;
        Isinv = Cbs'* aircraft.Ibinv*Cbs; %ojito los cambios

    % forces equaation
        y1 = -aircraft.m * (Vrelwdot + cross(Omegaw_b_w,Vrelw)) + Fnetw - aircraft.m*cross(Omegaw,Vrelw);

    % moments equation expressed in stability frame
        y3 = -Omegasdot-cross(Omegas_b_s,Omegas) + Isinv*(Mnets-cross(Omegas,Is*Omegas));

    % assemble y vector
        y = [y1;y2;y3];
end
