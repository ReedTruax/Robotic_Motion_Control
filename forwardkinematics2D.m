%% Author RCT 2/28/2020

clc, clear, close all, format compact
L=[5 2 1 1]; %robotic linkage lengths
unit='d'; %'d' for degree 'r' for radian
thigh=[180 360 360 360]; %rotational angle of each joint
tlow=[0 0 0 0];
theta=[90, -90, 35, 40;
        90 90 90 90]; %degree/radian of each joint
start=[0,0]; %x,y starting position

robot=create_robot(L,tlow,thigh,start,unit,theta);

dat=forkin(robot,robot.theta)%0,robot.theta,robot.L,'d');

figure(1)
plot(dat(6:10,1),dat(6:10,2),'r-','LineWidth',3); hold on
plot(dat(6:10,1),dat(6:10,2),'bO','LineWidth',3);
ylim([-8,8]);xlim([-5,5]);
title('L_1=5, L_2=2, L_3=1, L_4=1')
hold off

[space]=showrange(robot,25);
% [space1]=showrange(robot.L,robot.trange,'d',robot.start,10);
% [space2]=showrange(robot.L,robot.trange,'d',robot.start,5);
% 
% 
% figure(2)
% subplot(1,3,1)
% plot(space(:,1),space(:,2),'*')
% title('Working Area, (\theta_{high}-\theta_{low})/25 L_1=5, L_2=2, L_3=1, L_4=1')
% subplot(1,3,2)
% plot(space1(:,1),space1(:,2),'*')
% title('Working Area, (\theta_{high}-\theta_{low})/10 L_1=5, L_2=2, L_3=1, L_4=1')
% subplot(1,3,3)
% plot(space2(:,1),space2(:,2),'*')
% title('Working Area, (\theta_{high}-\theta_{low})/5 L_1=5, L_2=2, L_3=1, L_4=1')


idat=inversekin(robot.L,[3.0780,6.5395],robot.start,0.001);
idat=forkin(robot,idat(:,3:6));
figure(2)
plot(idat(:,1),idat(:,2),'r-','LineWidth',3); hold on
plot(idat(:,1),idat(:,2),'bO','LineWidth',3);
ylim([-1,8]);xlim([-1,5]);
title('L_1=5, L_2=2, L_3=1, L_4=1')
hold off
%% Functions

function [dat]=inversekin(L,pos,start,errtarg)
%Inputs: L=vector or arm lengths, pos=vector of interested positions
% Optional Inputs: start [x,y] base position if empty assumed to be 0,0
% errtarg=target error between given final pos and actual calculated
% position
%return joint posistions in deg

if~exist('errtarg','var')
    errtarg=0.01;
end

if ~exist('start','var')
    start=[0,0];
end



%define general var
jointnum=length(L);

%set every maximum joint angle to 360, and generate postions with every
%joint combination from 0 to 360 in 100 increments
trange=360*ones(jointnum,1);
space=showrange(L,trange,'d',start,25);

%create data matrix for x,y and joint angle positions
dat=zeros(length(pos(:,1)),length(space(1,:))+1); 

k=1;

for i=1:length(pos(:,1))
    %if i==1
        %find minimum x,y pos err from list
        err=abs(space(:,1)-pos(i,1))/pos(i,1)+abs(space(:,2)-pos(i,2))/pos(i,2);
        errmin = min(err(:));
        [row] = find(err==errmin);
        %take the corresponding joint positions for the minimum angle
        dat(i,:)=[space(row(1),:),errmin];
        
        while errmin>errtarg
          tl=dat(i,3:3+jointnum-1)*1-2/(10^(k));
          th=dat(i,3:3+jointnum-1)*1+2/(10^(k));
          spacetemp=showrange(L,th,'d',start,20,tl);
          space=horzcat(space',spacetemp')';
          err=abs(space(:,1)-pos(i,1))/pos(i,1)+abs(space(:,2)-pos(i,2))/pos(i,2);
          errmin = min(err(:));
          [row] = find(err==errmin);
          dat(i,:)=[space(row(1),:),errmin];
          k=k+1;
          if k>10
              error('%f, %f not with robotic range',pos(i,1),pos(i,2))
          end
        end
        k=1;
end
end


function [space]=showrange(robot,ptsnum,th,tl, L,unit,start)
% shows the spatial range the linkages can encompase
 
%Inputs:
% robot: (optional) data matrix defining the robot if unused pass a 0 but
% must pass L, th and unit seperately
% ptsnum: (integer or array) if integer is the number of points user wants 
%         tl-th divided by.  If array each joint has corresponding value.
%         Defaults to 15 if no value supplied
% th      Only required if no robot structure, array of upper bounds on
%         joint rotation, if present will overide the robot structure th value
%tl       (optional) matrix of lower bounds on joint angles, if unspecified
%         tl assumed to be 0 for all joints, if present will overide the robot structure th value
% L       (optional) matrix of linkage lengths 
% unit    (optional) 'r' for radians, 'd' for degrees, if nothing is entered assumed to be radians
%start    (optional) 2x1 matrix of for robot origin, if unspecified assumed to be 0

%Returns x,y, position of end effector and associated joint angles

%Checks passed vairables
if isa(robot,'struct')
    L=robot.L;
    start=robot.start;
    unit=robot.unit;
    if exist('th','var')
        if length(th)==length(L)
            
        else
            th=robot.thigh;
        end
    else
        th=robot.thigh;
    end
    if exist('tl','var')
        if length(tl)==length(L)
            
        else
            tl=robot.tlow;
        end
    else
        tl=robot.tlow;
    end
else
    if ~exist('L','var')
        error('Robot structure not passed, need to pass L');
    end
       
    if ~exist('th','var')
        error('Need to input Th vector if not using robot data structure')
    end
    
    if ~exist('start','var')
        start=[0,0];
    end

    if ~exist('unit','var')
        unit='r';
    end
    if ~exist('tl','var')
        tl=zeros(length(L),1);
    end
    
    if length(L)~=length(th) && length(L)~=length(tl)
        error('L, Th, Tl must all be the same length')
    end
end

if ~exist('ptsnum','var')
    ptsnum=15;
else
    if length(ptsnum)==1
        val=ptsnum;
        ptsnum=ones(length(L),1)*val;
    elseif a~=length(L)
        error('Length of ptsnum needs to either be the same length as joint array or an integer');
    end
end


len=length(L);

pts=1;
for i=1:len
    pts=pts*(ptsnum(i));
end

x=zeros(pts,1);
y=zeros(pts,1); 
ang=zeros(pts,len+1); %tracks pos of each point

k=1;

%creates a matrix of angles where each joint range is divded corresponding ptsnum
tspace=zeros(pts,length(L)); 

for i=1:length(th)
    tspace(:,i)=linspace(tl(i),th(i),ptsnum(i))';
end

%goes through every permutation and calculates the x,y,ang of the end
%affector
pos=ones(len,1); pos(len)=0;
if len==1
    ptsnum=ptsnum-1;
end

theta=zeros(len,1);
while pos(1)~=ptsnum+1
    pos(len)=pos(len)+1;
    for i=1:len
        if len>1
            if pos(i)==ptsnum+1
                pos(i)=1;
                pos(i-1)=pos(i-1)+1;
            end
        end
        theta(i)=tspace(pos(i),i);    
    end
    [dat]=forkin(0,theta',L,unit,start);
    x(k)=dat(end,1); y(k)=dat(end,2); ang(k,:)=dat(:,3)';
    k=k+1;
    
end
space=[x,y,ang];
end


function [dat]=forkin(robot,theta,L,unit,start)
%Computes the X,Y, Theta of the end locations of an N linkage robotic arm. 
%Inputs:
% robot: (optional) data matrix defining the robot if unused pass a 0 but
% must pass L, and unit seperately
% theta: matrix of joint angles, must be of same length as L, this can be a part of robot structure
% L     (optional) matrix of linkage lengths 
% unit  (optional) 'r' for radians, 'd' for degrees, if nothing is entered assumed to be radians
%start  (optional) 2x1 matrix of for robot origin, if unspecified assumed
%to be 0
%Returns x,y, theta position of end effector

%Checks passed vairables
if isa(robot,'struct')
    L=robot.L;
    if isfield(robot,'start')==0
        start=[0,0];
    else
        start=robot.start;
    end
    if isfield(robot,'unit')==0
        unit='r';
    else
        unit=robot.unit;
    end
    if isfield(robot,'theta')==0
        
    else
        theta=robot.theta;
    end
else
    if ~exist('start','var')
        start=[0,0];
    end

    if ~exist('unit','var')
        unit='r';
    end
    
    
end


%Ensures L and theta matrixes are the same size
if length(L)~=length(theta)
    error('L and theta not of same length')
end

% if L is of size 0 report start point, if no start point specified,
% defaults to 0,0
if isempty(L)==1
    x=start(1);
    y=start(2);
    return;
end
b=size(theta);

%Prealocate joint position matrices
x=zeros(1,(length(L)+1)*b(1));
y=zeros(1,(length(L)+1)*b(1));
thetasol=zeros(1,(length(L)+1)*b(1));
con=1;

for j=1:b(1)
    %for each set of angles set the first position to be the start position
    thetasol(con)=theta(j,1);
    x(con)=start(1);
    y(con)=start(2);
    con=con+1;
   
    for i=1:length(L)
        if unit=='r'
            if i<= length(L)-1
                thetasol(con)=thetasol(con-1)+theta(j,i+1);
            else
                thetasol(con)=thetasol(con-1);
            end
            x(con)=x(con-1)+cos(thetasol(con-1))*L(i);
            y(con)=y(con-1)+sin(thetasol(con-1))*L(i);
            con=con+1;
        elseif unit=='d'
            if i<= length(L)-1
                thetasol(con)=thetasol(con-1)+theta(j,i+1);
            else
                thetasol(con)=thetasol(con-1);
            end
            x(con)=x(con-1)+cosd(thetasol(con-1))*L(i);
            y(con)=y(con-1)+sind(thetasol(con-1))*L(i);
            con=con+1;
        end
    end
    

end
dat=[x;y;thetasol]';
end

function rob=create_robot(L,tlow,thigh,start,unit,theta)
%This function creates a robot structure 
%Inputs:
% L     matrix of linkage lengths 
% tlow  matrix of lower bounds on joint rotation, must be of same length as L
% thigh matrix of upper bounds on joint rotation, must be of same length as L
% start (optional) 2x1 matrix of for robot origin, if unspecified assumed to be 0,0
% unit  (optional) 'r' for radians, 'd' for degrees, if nothing is entered assumed to be radians
% theta (optional) list of specific joint angles, can be a 1D list or
% matrix if theta is a matrix must be of size mxn where n is the number of
% joints 

%Returns robot structure


if length(L)~=length(tlow)&&length(L)~=length(thigh)
    error('L, Tlow, Thigh must be of same length')
end

if ~exist('L','var')
    error('Please input a vector of linkage lengths')
else
    rob.L=L;
end

if ~exist('tlow','var')
    error('Please input a vector of lower rotation bounds')
else
    rob.tlow=tlow;
end

if ~exist('thigh','var')
    error('Please input a vector of upper rotation bounds')
else
    rob.thigh=thigh;
end

if ~exist('start','var')
    rob.start=[0,0];
else
    rob.start=start;
end

if ~exist('unit','var')
    rob.unit='r';
else
    rob.unit=unit;
end

if ~exist('theta','var')
    rob.theta=zeros(length(L),1);
else
    b=size(theta);
    if b(2)~=length(L)
        error('Number of columns of theta does not match number of joints')
    else
        rob.theta=theta;
    end
end


end


