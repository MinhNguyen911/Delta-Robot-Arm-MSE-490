%1.1. Define size of figure and create figure handle (DO NOT MODIFY)


set(0,'Units','pixels');
dim = get(0,'ScreenSize'); 
fig_handle = figure('doublebuffer','on','Position',[0,35,dim(3),dim(4)-100],...
    'Name','3D Object','NumberTitle','off');
set(gcf,'color', [1 1 1]) %Background Colour

%1.2 Define the light in the figure (CHANGE POSITION VECTOR IF FIGUPP IS TOO BRIGHT/DARK)
set(fig_handle,'renderer','zbuffer','doublebuffer','off')
light('color',[.5,.5,.5],'position',[0,1,3],'Style','infinite')
lighting gouraud
daspect([1 1 1]);
axis off

%Arrows (CHANGE PARAMETERS IF THEY ARE TOO SMALL OR TOO BIG)
% You need to have the file arrow3 in the same directory
   arrow_length=1; hold on
   line([0,0],[0,0], [0,arrow_length]); text(0,0,arrow_length*1.5,'z_0','FontSize',14); 
   line([0,0],[0,arrow_length],[0,0]); text(0,arrow_length*1.5, 0,'y_0','FontSize',14); 
   line([0,arrow_length],[0,0],[0,0]); text(arrow_length*1.5, 0, 0,'x_0','FontSize',14); 

   axis ([-1000,1000,-1000,1000,-1000,1000]);
   
   view(30,20)
   
% Convert figure into Object (LOAD YOUR PARTS)       
load('Base_Platform.mat');
setappdata(0,'object_data',object);
object = getappdata(0,'object_data');
obj{1}=object;

load('mobile_gripper.mat');
setappdata(0,'object_data',object);
object = getappdata(0,'object_data');
obj{2}=object; 
%%%%%%% ARM 1
load('Upper_Link_1.mat');
setappdata(0,'object_data',object);
object = getappdata(0,'object_data');
obj{3}=object; 

load('Lower_Link_1.mat');
setappdata(0,'object_data',object);
object = getappdata(0,'object_data');
obj{4}=object;

load('Lower_Link_2.mat');
setappdata(0,'object_data',object);
object = getappdata(0,'object_data');
obj{5}=object;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%% ARM 2
load('Upper_Link_2.mat');
setappdata(0,'object_data',object);
object = getappdata(0,'object_data');
obj{6}=object; 

load('Lower_Link_3.mat');
setappdata(0,'object_data',object);
object = getappdata(0,'object_data');
obj{7}=object; 

load('Lower_Link_4.mat');
setappdata(0,'object_data',object);
object = getappdata(0,'object_data');
obj{8}=object;

%%%%%%% ARM 2
load('Upper_Link_3.mat');
setappdata(0,'object_data',object);
object = getappdata(0,'object_data');
obj{9}=object;

load('Lower_Link_5.mat');
setappdata(0,'object_data',object);
object = getappdata(0,'object_data');
obj{10}=object; 

load('Lower_Link_6.mat');
setappdata(0,'object_data',object);
object = getappdata(0,'object_data');
obj{11}=object; 

q=zeros(11);
for i=1:11%(CHANGE if you have 10 parts, change it to i=1:10)
    q(i) = patch('faces', obj{i}.F, 'vertices', obj{i}.V);
    set(q(i),'EdgeColor','none');
end

%Set colour to the componenets (CHANGE colours of new parts)
set(q(1),'FaceColor',  [1,0.242,0.293]);
set(q(2),'FaceColor',  [1,0.8,0.5]);

set(q(3),'FaceColor',  [.3,0.5,0.3]);
set(q(4),'FaceColor',  [.5,0.7,0.5]);
set(q(5),'FaceColor',  [.5,0.7,0.5]);

set(q(6),'FaceColor',  [.3,0.5,0.3]);
set(q(7),'FaceColor',  [.5,0.7,0.5]);
set(q(8),'FaceColor',  [.5,0.7,0.5]);

set(q(9),'FaceColor',  [.3,0.5,0.3]);
set(q(10),'FaceColor', [.5,0.7,0.5]);
set(q(11),'FaceColor', [.5,0.7,0.5]);

%NEW


%ANIMATION
RGB=256; %Resolution
fm = getframe; 
[img,map] = rgb2ind(fm.cdata,RGB,'nodither'); 

load('joints');
load('position');
%KINEMATICS
%Characteristics of the manipulator
  % Definition of base platform
L=0.25*1000;                        % base length (centPP to joint)
LL=0.1*1000;                      % mobile length (centPP to joint)
P=0.45*1000;                        % Link first length
PP =0.45*1000;                    % Length second Length
syms x y z
%%%%%%%%%%%%%%%%%%%%
%INVERSE KINEMATICS%
%%%%%%%%%%%%%%%%%%%%

%%x0=0.05*1000; y0=0.2*1000; z0=-0.5*1000;  %Position of Mobile Platform
P_ee=[0	-150	-150	-150	-50	-50	-50	0	0	0	0	0	0	150	150	150	50	50	50	0;
    0	-200	-200	-200	200	200	200	-200	-200	-200	200	200	200	-200	-200	-200	200	200	200	0;
    -500	-500	-700	-500	-500	-700	-500	-500	-700	-500	-500	-700	-500	-500	-700	-500	-500	-700	-500	-500];
tf=ones(1,length(P_ee(1,:))-1)*0.5;

% joints=zeros(9,length(P_ee(1,:)));
% for i=1:length(P_ee(1,:))
% 
% joints(1:9,i)=inverse_kin(P_ee(1,i),P_ee(2,i),P_ee(3,i));
% 
% end

% TRAJECTORY GENERATION. Trajectory generation of all the joints
% dt=0.1; %stepsize
% [position,velocity,acceleration,time]=via_points_match_VA(joints, tf, dt, 'prescribed',[0,0]); 

 %ANIMATION (DO NOT CHANGE)
 n=length(position(1,:));
 mov(1:length(n)) = struct('cdata', [],'colormap', []);
 [a,b]=size(img); gifim=zeros(a,b,1,n-1,'uint8'); 
% m=1;
% while (m< (n+4)/6)
%     [X1]=forward_kin(position(1,6*m-4),position(2,6*m-4),position(3,6*m-4));
%     [X2]=forward_kin(position(1,6*m-3),position(2,6*m-3),position(3,6*m-3));
%     [X3]=forward_kin(position(1,6*m-2),position(2,6*m-2),position(3,6*m-2));
%     [X4]=forward_kin(position(1,6*m-1),position(2,6*m-1),position(3,6*m-1));
%     [angle1]=inverse_kin(X1(1),X1(2),X1(3));
%     [angle2]=inverse_kin(X2(1),X2(2),X2(3));
%     [angle3]=inverse_kin(X3(1),X3(2),X3(3));
%     [angle4]=inverse_kin(X4(1),X4(2),X4(3));
%     for i=4:9
%         position(i,6*m-4)=angle1(i);
%         position(i,6*m-3)=angle2(i);
%         position(i,6*m-2)=angle3(i);
%         position(i,6*m-1)=angle4(i);
%     end
%     m=m+1;
% end
    
 % FORWARD KINEMATICS
 % NEW LOOP
 for k=1:n %Make sure you don’t use k
     
     theta11=position(1,k);
     theta12=position(2,k);
     theta13=position(3,k);
     
     theta21=position(4,k);
     theta22=position(5,k);
     theta23=position(6,k);
     
     theta31=position(7,k);
     theta32=position(8,k);
     theta33=position(9,k);
     [XYZ]=forward_kin(theta11,theta12,theta13);
     
     offset=50;
     R1Z=[0 1 0; -1 0 0;0 0 1]; %rotate -90 degree
     R1Y=[0 0 -1;0 1 0;1 0 0]; % rotate -90
     R1Z_2=[cosd(120) -sind(120) 0; sind(120) cosd(120) 0; 0 0 1]; % Rotate 120 degree in Z
     
     J1=[0,-L-P*cos(abs(theta11)),-P*sin(abs(theta11))];   %Point J1, J2, J3
     J2=[(L+P*cos(abs(theta12)))*cos(pi/6),(L+P*cos(abs(theta12)))*sin(pi/6),-P*sin(abs(theta12))];
     J3=[-(L+P*cos(abs(theta13)))*cos(pi/6),(L+P*cos(abs(theta13)))*sin(pi/6),-P*sin(abs(theta13))];
     %%%%% ARM 1 2 3 T matrices
     T1_1=[cos(theta11) -sin(theta11) 0 0; sin(theta11) cos(theta11) 0 0; 0 0 1 0; 0 0 0 1];
     T2_1=[cos(theta21) -sin(theta21) 0 P; sin(theta21) cos(theta21) 0 0; 0 0 1 0; 0 0 0 1];
     T3_1=[cos(theta31) sin(theta31) 0 0; 0 0 -1 0; sin(theta31) cos(theta31) 0 0; 0 0 0 1];
     
     T1_2=[cos(theta12) -sin(theta12) 0 0; sin(theta12) cos(theta12) 0 0; 0 0 1 0; 0 0 0 1];
     T2_2=[cos(theta22) -sin(theta22) 0 P; sin(theta22) cos(theta22) 0 0; 0 0 1 0; 0 0 0 1];
     T3_2=[cos(theta32) sin(theta32) 0 0; 0 0 -1 0; sin(theta32) cos(theta32) 0 0; 0 0 0 1];
     
     T1_3=[cos(theta13) -sin(theta13) 0 0; sin(theta13) cos(theta13) 0 0; 0 0 1 0; 0 0 0 1];
     T2_3=[cos(theta23) -sin(theta23) 0 P; sin(theta23) cos(theta23) 0 0; 0 0 1 0; 0 0 0 1];
     T3_3=[cos(theta33) sin(theta33) 0 0; 0 0 -1 0; sin(theta33) cos(theta33) 0 0; 0 0 0 1];
     
     for i=1:11 %(CHANGE n=2 for the number of parts that you have)
        V{i} = obj{i}.V'; 
    end
     
     
     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
     %Position of End Effector
     newV{1} = V{1} + repmat([0,0,0]',[1 length(V{1}(1,:))]); %The position of the base
     
     newV{2} = V{2} + repmat([XYZ(1),XYZ(2),XYZ(3)]',[1 length(V{2}(1,:))]); %The position of the mobile platform
     
     newV{3}=  R1Y*R1Z*T1_1(1:3,1:3)*V{3};   % ARM 1 Upper LINK
     newV{3}=newV{3}+ repmat([0;-L;0],[1 length(newV{3}(1,:))]);
     
     newV{4}= (R1Y*R1Z*T1_1(1:3,1:3))*T2_1(1:3,1:3)*T3_1(1:3,1:3)*V{4};  % ARM 1 Lower LINK
     newV{4}=newV{4}+ repmat([J1(1)+offset;J1(2);J1(3)],[1 length(newV{4}(1,:))]);
     
     newV{5}= (R1Y*R1Z*T1_1(1:3,1:3))*T2_1(1:3,1:3)*T3_1(1:3,1:3)*V{5};   % ARM 1 Lower LINK
     newV{5}=newV{5}+ repmat([J1(1)-offset;J1(2);J1(3)],[1 length(newV{5}(1,:))]);
     
     newV{6}=R1Z_2*R1Y*R1Z*T1_2(1:3,1:3)*V{6};   % ARM 2 Upper LINK
     newV{6}=newV{6}+ repmat([L*cosd(30);L*sind(30);0],[1 length(newV{6}(1,:))]);
     
     newV{7}= R1Z_2*(R1Y*R1Z*T1_2(1:3,1:3))*T2_2(1:3,1:3)*T3_2(1:3,1:3)*V{7};  % ARM 2 Lower LINK
     newV{7}=newV{7}+ repmat([J2(1)+offset*cosd(60);J2(2)-offset*sind(60);J2(3)],[1 length(newV{7}(1,:))]);
     
     newV{8}= R1Z_2*(R1Y*R1Z*T1_2(1:3,1:3))*T2_2(1:3,1:3)*T3_2(1:3,1:3)*V{8};   % ARM 2 Lower LINK
     newV{8}=newV{8}+ repmat([J2(1)-offset*cosd(60);J2(2)+offset*sind(60);J2(3)],[1 length(newV{8}(1,:))]);
     
     newV{9}=R1Z_2*R1Z_2*R1Y*R1Z*T1_3(1:3,1:3)*V{9};  % ARM 3 Upper LINK
     newV{9}=newV{9}+ repmat([-L*cosd(30);L*sind(30);0],[1 length(newV{9}(1,:))]);
     
     newV{10}= R1Z_2*R1Z_2*(R1Y*R1Z*T1_3(1:3,1:3))*T2_3(1:3,1:3)*T3_3(1:3,1:3)*V{10};  % ARM 3 Lower LINK
     newV{10}=newV{10}+ repmat([J3(1)-offset*cosd(60);J3(2)-offset*sind(60);J3(3)],[1 length(newV{10}(1,:))]);
     
     newV{11}= R1Z_2*R1Z_2*(R1Y*R1Z*T1_3(1:3,1:3))*T2_3(1:3,1:3)*T3_3(1:3,1:3)*V{11};   % ARM 3 Lower LINK
     newV{11}=newV{11}+ repmat([J3(1)+offset*cosd(60);J3(2)+offset*sind(60);J3(3)],[1 length(newV{11}(1,:))]);
   
    for ii=1:11 %(CHANGE n=2 to the number of parts that you have)
    set(q(ii),'Vertices',newV{ii}(1:3,:)'); %Set the new position in the handle (graphical link)
    end    
    drawnow    
    im= frame2im(getframe);
     gifim(:,:,:,k) = rgb2ind(im, map);
     mov(k)=getframe(gcf);
 end
 
% Close loop k
%ANIMATION, creates animated gif (DO NOT MODIFY)
imwrite(gifim,map,'Project.gif','DelayTime',0)%,'LoopCount',inf) 



