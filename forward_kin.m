function [XYZ] = forward_kin(theta11,theta12,theta13)
%Characteristics of the manipulator
L=0.25*1000;                        % base length (centPP to joint)
LL=0.1*1000;                      % mobile length (centPP to joint)
P=0.45*1000;                        % Link first length
PP =0.45*1000;                    % Length second Length
% %FORWARD KINEMATICS%
%%%%%%%%%%%%%%%%%%%%
syms x y z
J1=[0,-L-P*cos(abs(theta11)),-P*sin(abs(theta11))];   %Point J1, J2, J3
J2=[(L+P*cos(abs(theta12)))*cos(pi/6),(L+P*cos(abs(theta12)))*sin(pi/6),-P*sin(abs(theta12))];
J3=[-(L+P*cos(abs(theta13)))*cos(pi/6),(L+P*cos(abs(theta13)))*sin(pi/6),-P*sin(abs(theta13))];
E1E0=[0,LL,0]; %Vector E1E0, E2E0, E3E0
E2E0=[-LL*cos(pi/6),-LL*sin(pi/6),0];
E3E0=[LL*cos(pi/6),-LL*sin(pi/6),0];
J1_prime=E1E0+J1;   %Point J1',J2',J3'
J2_prime=E2E0+J2;
J3_prime=E3E0+J3;
[solx,soly,solz]=solve(x^2+(y-J1_prime(2))^2+(z-J1_prime(3))^2==PP^2,...
    (x-J2_prime(1))^2+(y-J2_prime(2))^2+(z-J2_prime(3))^2==PP^2,...
    (x-J3_prime(1))^2+(y-J3_prime(2))^2+(z-J3_prime(3))^2==PP^2);
index=find(solz==min(solz(:)));
XYZ=[double(solx(index));double(soly(index));double(solz(index))];
end