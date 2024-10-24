% trimConditionsPullup.m
% function to calculate steady pull up / pull over flight for a fixed wing aircraft
%usage
% function [alpha,deltat,ih,deltae,fval,flag] = trimConditionsPullup(V,h,pitchRate,deltaCGb,aircraft)
% where
    % V
    % h
    % pitchRate = pitch rade (rad/s)
    % deltaCGb
    % aircraft
    % alpha :  angle of attack (rad)
    % deltat : propulsion system control (0<=deltat<=1)
    % ih : horizontal control incidence control (rad)
    % deltae : elevator (rad)
    % fval : value of ftrimRect (Xi, h, Vvert, deltaCGb, aircraft) in calculate conditions
    % flag :  flag to calculate how the numerical optimization method
    % exited


function [alpha,deltat,ih,deltae,fval,flag] = trimConditionsPullup(V,h,pitchRate,deltaCGb,aircraft)

    %calculate steady linear flight conditios for this aircraft

    [theta,gamma,alpha,deltat,ihTrimRect,deltae,fval,flag] = trimConditionsRect(V,h,0,deltaCGb,aircraft);



    % initial guess
    alpha0 = 0;
    deltat0 = 0.5;
    pitchControl0 = 0;
    Xi0 = [alpha0;deltat0;pitchControl0];

    % lower bounds for Xi
    alphamin = -5*pi/180;
    deltatmin = 0;
    pitchControlmin = -20*pi/180;
    lb = [alphamin;deltatmin;pitchControlmin];

    % upper bounds for Xi
    alphamax = 12*pi/180;
    deltatmax = 1;
    pitchControlmax = 20*pi/180;
    ub = [alphamax;deltatmax;pitchControlmax];

    %numerical optimization method parameters
    maxIter = 100000;
    tol = 1e-9;



    if  isOctave()
        %for octave
        [Xitrim, fval,flag] = sqp(Xi0, @(Xi) ftrimPullup(Xi,V,h,pitchRate,ihTrimRect,deltaCGb,aircraft),[], [], lb, ub, maxIter, tol);
    else
       %for Matlab
       % set options for numerical optimizaion method
      % options = optimset('fmincon', 'Display', 'off', 'Algorithm','sqp', 'MaxIter', maxIter, 'TolX', tol,'TolFun',tol);
        options = optimset('Display', 'off', 'Algorithm','sqp', 'MaxIter', maxIter, 'TolX', tol,'TolFun',tol);
       [Xitrim, fval,flag] = fmincon(@(Xi) ftrimPullup(Xi,V,h,pitchRate,ihTrimRect,deltaCGb,aircraft),Xi0,[],[],[],[],lb,ub,[],options);
    end

    % Extract data of Xitrim
    alpha = Xitrim(1,1);
    deltat = Xitrim(2,1);
    pitchControl = Xitrim(3,1);

    if aircraft.Cmdeltae ~= 0;
        ih = ihTrimRect; %Situciones donde la aeronave solo tiene control de incidencia, o tiene los dos controles, por tanto uso el control de incidencia de la cola horizontal
        deltae = pitchControl; %como el control de trim
    else
        ih = pitchControl;
        deltae = 0;
    end


end


