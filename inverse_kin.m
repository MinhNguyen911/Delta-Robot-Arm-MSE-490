function [J]=inverse_kin(x0,y0,z0)
%KINEMATICS
%Characteristics of the manipulator
L=0.25*1000;                        % base length (centPP to joint)
LL=0.1*1000;                      % mobile length (centPP to joint)
P=0.45*1000;                        % Link first length
PP =0.45*1000;                    % Length second Length
syms x y z

%%%%%%%%%%%%%% SOLVE FOR THETA 1 ANGLES %%%%%%%%%%%%%%%%%%%%
%%%%Branch 1
xyz_o1=[x0,y0,z0];  %%%% LOCATION OF THE MOVING PLATFORM: INPUT HEPP
E1_prime=[0,xyz_o1(2)-LL,xyz_o1(3)]; % Point E1'
F1=[0,-L,0];   %Point F1

    %Solving for the intersection of 2 circles
[soly,solz]=solve((y-F1(2))^2+(z-F1(3))^2-P^2==0,(y-E1_prime(2))^2+(z-E1_prime(3))^2-PP^2+xyz_o1(1)^2==0);
%Choose the smaller set of solution
index=find(soly == min(soly(:)));
y1=soly(index); 
z1=solz(index);

theta11=double(atan2(z1,F1(2)-y1));  %Solve for theta11
%%%%Branch 2
xyz_o2=[xyz_o1(1)*cos(2*pi/3)+xyz_o1(2)*sin(2*pi/3),-xyz_o1(1)*sin(2*pi/3)+xyz_o1(2)*cos(2*pi/3),xyz_o1(3)];
E2_prime=[0,xyz_o2(2)-LL,xyz_o2(3)]; % Point E2'
F2=[0,-L,0];                       % Point F2
%Solve for intersection of 2 circles
[soly,solz]=solve((y-F2(2))^2+(z-F2(3))^2-P^2==0,(y-E2_prime(2))^2+(z-E2_prime(3))^2-PP^2+xyz_o2(1)^2==0);
%Choose the smaller set of solution
index=find(soly == min(soly(:)));
y2=soly(index); 
z2=solz(index);

theta12=double(atan2(z2,F2(2)-y2)); %Solve for theta12

%%%%Branch 3
xyz_o3=[xyz_o1(1)*cos(-2*pi/3)+xyz_o1(2)*sin(-2*pi/3),-xyz_o1(1)*sin(-2*pi/3)+xyz_o1(2)*cos(-2*pi/3),xyz_o1(3)];
E3_prime=[0,xyz_o3(2)-LL,xyz_o3(3)];% Point E3'
F3=[0,-L,0];                      % Point F3
%Solve for intersection of 2 circles
[soly,solz]=solve((y-F3(2))^2+(z-F3(3))^2-P^2==0,(y-E3_prime(2))^2+(z-E3_prime(3))^2-PP^2+xyz_o3(1)^2==0);
%Choose the smaller set of solution
index=find(soly == min(soly(:)));
y3=soly(index);   
z3=solz(index);  

theta13=double(atan2(z3,F3(2)-y3)); %Solve for theta13

R=[0 -1 0; 0 0 1; -1 0 0]; %Rotation Matrix
syms theta1 theta2 theta3
%% Branch 1
OE=[(xyz_o1(1)-0);(xyz_o1(2)-LL--L);(xyz_o1(3)-0)];
OE_prime=R*OE; 
[q1,w1,e1]=solve((P*cos(theta1) + PP*cos(theta3)*(cos(theta1)*cos(theta2) - sin(theta1)*sin(theta2)))== OE_prime(1),...
    P*sin(theta1) + PP*cos(theta3)*(cos(theta1)*sin(theta2) + cos(theta2)*sin(theta1))==OE_prime(2),...
    (PP*sin(theta3))==OE_prime(3));
q1=double(q1);
w1=double(w1);
e1=double(e1);
for i=1:1:4
    if (abs(q1(i)-theta11)<10^-3)

        theta21=w1(i);
        theta31=e1(i);
        break
    end
   
end
%% Branch 2
OE2=[xyz_o1(1)*cos(2*pi/3)+xyz_o1(2)*sin(2*pi/3);-xyz_o1(1)*sin(2*pi/3)+xyz_o1(2)*cos(2*pi/3)-LL--L;OE(3)];
OE2_prime=R*OE2;
[q2,w2,e2]=solve((P*cos(theta1) + PP*cos(theta3)*(cos(theta1)*cos(theta2) - sin(theta1)*sin(theta2)))== OE2_prime(1),...
    P*sin(theta1) + PP*cos(theta3)*(cos(theta1)*sin(theta2) + cos(theta2)*sin(theta1))==OE2_prime(2),...
    (PP*sin(theta3))==OE2_prime(3));
q2=double(q2);
w2=double(w2);
e2=double(e2);
for i=1:1:4
    if (abs(q2(i)-theta12)<10^-3)
  
        theta22=w2(i);
        theta32=e2(i);
       break
    end
end
% Branch 3
OE3=[xyz_o1(1)*cos(-2*pi/3)+xyz_o1(2)*sin(-2*pi/3);-xyz_o1(1)*sin(-2*pi/3)+xyz_o1(2)*cos(-2*pi/3)-LL--L;OE(3)];
OE3_prime=R*OE3;
[q3,w3,e3]=solve((P*cos(theta1) + PP*cos(theta3)*(cos(theta1)*cos(theta2) - sin(theta1)*sin(theta2)))== OE3_prime(1),...
    P*sin(theta1) + PP*cos(theta3)*(cos(theta1)*sin(theta2) + cos(theta2)*sin(theta1))==OE3_prime(2),...
    (PP*sin(theta3))==OE3_prime(3));
q3=double(q3);
w3=double(w3);
e3=double(e3);
for i=1:1:4
    if (abs(q3(i)-theta13)<10^-3)

        theta23=w3(i);
        theta33=e3(i);
        break
    end
end
J=[theta11;theta12;theta13;theta21;theta22;theta23;theta31;theta32;theta33];
end
