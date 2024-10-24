% Cbsmatrix.m
% function to calculate rotation matrix from relative wind frame to body frame
% Usage 
%   Where:
%     alpha : angle of attack (rad)
%     %     Cbs  : rotation matrix from stability frame to body frame

function Cbs = Cbsmatrix(alpha)
    beta = 0;
    %Calculate cosine and sine of alpha 
    calpha = cos(alpha);
    salpha = sin(alpha);


    % calculate Cbw

    Cbs = [calpha -calpha*0  -salpha;
            0       1           0;
            salpha  0 calpha];
end


%EJEMPLO DE LLAMAR RESULTRADOS 