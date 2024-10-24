% linearizeAircraftModel.m
% function to linearize aircraft model given by
%   f(xdot,x,u,h,deltaCGb,aircraft) = 0
%
%   Usage
%       [E,A,B,Elon,Alon,Blon,Alonprime,Blonprime,Elat,Alat,Blat,Alatprime,Blatprime,eigLon,eigLat,x0,u0] = linearizeAircraftModel(V,h,deltaCGb,aircraft)
%       where
%           E, A, B   :  matrices for the linearizaded model written as E*xdot  = A*x + B*u
%
%           Elon, Alon, Blon
%           Alonprime,  Blonprime
%           Elat, Alat, Blat  :
%           Alatprime,  Blatprime
%           eigLon
%           eigLat
%           x0
%           u0
%
%
%
%
%
%


function [E,A,B,Elon,Alon,Blon,Alonprime,Blonprime,Elat,Alat,Blat,Alatprime,Blatprime,eigLon,eigLat,x0,u0] = linearizeAircraftModel(V,h,deltaCGb,aircraft)

    % calculate steady level rrectilinear flight condition
       [theta0,gamma,alpha0,deltat0,ih0,deltae0,fval,flag] = trimConditionsRect(V,h,0,deltaCGb,aircraft);

       % assemble xdot0, x0, u0  for steady level rectilinear flight

       xdot0 = [0;0;0;0;0;0;0;0;0];
       x0 = [V;0;alpha0;0;theta0;0;0;0;0];
       u0  = [deltat0;ih0;deltae0;0;0];


       % claculate f in steady condition f(xdot0,x0,u0,h,deltaCGb,aircraft)
       y0 = f(xdot0,x0,u0,h,deltaCGb,aircraft);

       % calculate jacobian matrices E,A, B
       dx  = 1e-6;
       E = zeros(9);
       A = zeros(9);


       for j= 1:9
          deltax  = zeros(9,1);
          deltax(j)  = dx;
          E(:,j)  = -(f(xdot0 + deltax,x0,u0,h,deltaCGb,aircraft)- y0)/dx;
          A(:,j)  = (f(xdot0,x0 + deltax,u0,h,deltaCGb,aircraft)- y0)/dx;

       end

       B = zeros(9,5);
       for j = 1:5
           deltax  =zeros(5,1);
           deltax(j) = dx;
           B(:,j)  = (f(xdot0,x0, u0 + deltax,h,deltaCGb,aircraft)- y0)/dx;

       end

       % Extrac Elong, Along, Blong matrices for longitudinal linearized
       % model
        Elon  = [E(1,:);E(3,:);E(5,:);E(8,:)];
        Elon  = [Elon(:,1), Elon(:,3),Elon(:,5), Elon(:,8)];


        Alon  = [A(1,:);A(3,:);A(5,:);A(8,:)];
        Alon  = [Alon(:,1), Alon(:,3),Alon(:,5), Alon(:,8)];

        Blon  = [B(1,:);B(3,:);B(5,:);B(8,:)];
        Blon = Blon(:,1:3);

       % calculate Alonprime Blonprime

       Alonprime = Elon\Alon;
       Blonprime = Elon\Blon;

       % Exteract Elat,  Alat, Blat, matrices for linearized lateral model
       Elat  = [E(2,:);E(4,:);E(7,:);E(9,:)];
       Elat  = [Elat(:,2), Elat(:,4),Elat(:,7), Elat(:,9)];


       Alat  = [A(2,:);A(4,:);A(7,:);A(9,:)];
       Alat  = [Alat(:,2), Alat(:,4),Alat(:,7), Alat(:,9)];

       Blat  = [B(2,:);B(4,:);B(7,:);B(9,:)];
       Blat = Blat(:,4:5);

       % calculate Alatprime Blatprime
       Alatprime = Elat\Alat;
       Blatprime = Elat\Blat;


       % calculate longitudinal eigenvalues
       eigLon = eig(Alonprime);
       [realEigLon, i] = sort(real(eigLon),'descend');
       eigLon = eigLon(i);

       % calclulate lateral values

       eigLat = eig(Alatprime);
       [realEigLat, i] = sort(real(eigLat),'descend');
       eigLat = eigLat(i);






end
