% trimConditionsRoll.m
% function to calculate steady turn flight for a fixed wing aircraft
% function [theta,alpha,beta,deltat,ih,deltae,deltaa,deltar,fval,flag] = trimConditionsRoll(V,h,rollRate,deltaCGb,aircraft)




%usage
% function [theta,alpha,beta,deltat,ih,deltae,deltaa,deltar,fval,flag] = trimConditionsRoll(V,h,rollRate,deltaCGb,aircraft)
% where
%      V : airspeed (m/s)
%      h : altitude (m)
%     rollRate
%      tunrRate: turn rate (rad/s)

%      deltaCGb : relative CG positions respect to nominal CG
%      aircraft : Data structure
%      phi = roll (rad)
%      theta : pich (rad)
%         gamma : flight path angle (rad)
%         alpha :  angle of attack (rad)
%         deltat : propulsion system control (0<=deltat<=1)
%         ih : horizontal control incidence control (rad)
%         deltae : elevator (rad)
%         deltar : ruder (rad)
%         deltaa : aileron (rad)
%         fval : value of fftrimTurn(Xi,V,h,Vvert,turnRate,ihTrimRect,deltaCGb,aircraft) in calculate conditions
%         flag :  flag to calculate how the numerical optimization method
%         exited

function [theta,alpha,beta,deltat,ih,deltae,deltaa,deltar,fval,flag] = trimConditionsRoll(V,h,rollRate,deltaCGb,aircraft)

    %calculate steady rectilinear flight conditiosn
    [theta,gamma,alpha,deltat,ihTrimRect,deltae,fval,flag] = trimConditionsRect(V,h,0,deltaCGb,aircraft);




    % initial guess
    theta0 = 0;
    alpha0 = 0;
    beta0 = 0;
    deltat0 = 0.5;
    pitchControl0 = 0; %OJO con esta varibale
    deltaa0 = 0;
    deltar0 = 0;

    Xi0 = [theta0;alpha0;beta0;deltat0;pitchControl0;deltaa0;deltar0];

    % lower bounds for Xi

    thetamin = -25*pi/180;
    alphamin = -5*pi/180;
    betamin = -10 *pi/180;
    deltatmin = 0;
    pitchControlmin = -20*pi/180;
    deltaamin = -20*pi/180;
    deltarmin = -20*pi/180;
    lb = [thetamin;alphamin;betamin;deltatmin;pitchControlmin;deltaamin;deltarmin];

    % upper bounds for Xi

    thetamax = 32*pi/180;
    alphamax = 12*pi/180;
    betamax = 10 *pi/180;
    deltatmax = 1;
    pitchControlmax = 20*pi/180;
    deltaamax = 20*pi/180;
    deltarmax = 20*pi/180;
    pitchControlmax = 20*pi/180;
    ub = [thetamax;alphamax;betamax;deltatmax;pitchControlmax;deltaamax;deltarmax];

    %numerical optimization method parameters
    maxIter = 100000;
    tol = 1e-9;



    if  isOctave()
        %for octave
        [Xitrim, fval,flag] = sqp(Xi0, @(Xi) ftrimRoll(Xi,V,h,rollRate,ihTrimRect,deltaCGb,aircraft),[],[],lb,ub,maxIter,tol);
    else
       %for Matlab
       % set options for numerical optimizaion method
      % options = optimset('fmincon', 'Display', 'off', 'Algorithm','sqp', 'MaxIter', maxIter, 'TolX', tol,'TolFun',tol);
        options = optimset('Display', 'off', 'Algorithm','sqp', 'MaxIter', maxIter, 'TolX', tol,'TolFun',tol);
       [Xitrim, fval,flag] = fmincon(@(Xi) ftrimRoll(Xi,V,h,rollRate,ihTrimRect,deltaCGb,aircraft),Xi0,[],[],[],[],lb,ub,[],options);
    end

    % Extract data of Xitrim

    theta  = Xitrim(1);
    alpha = Xitrim(2);
    beta = Xitrim(3);
    deltat = Xitrim(4);
    pitchControl = Xitrim(5);

    if aircraft.Cmdeltae ~= 0;
        ih = ihTrimRect; %Situciones donde la aeronave solo tiene control de incidencia, o tiene los dos controles, por tanto uso el control de incidencia de la cola horizontal
        deltae = pitchControl; %como el control de trim
    else
        ih = pitchControl;
        deltae = 0;
    end
      deltaa = Xitrim(6);
      deltar = Xitrim(7);


end


