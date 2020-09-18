% Reed Truax 2/11/2020
% Program is designed to take an image, convert it to x,y cordinates and 
% spit out path planning information for a robotic arm

clc, clear, close all, format compact

%% Defining constants

L=[5,3,2];
RGB=[0,0,0];
dat.page_size=[8.5,11];
dat.image_comp=20;


%% Read in the photo

[im_file,directory] = uigetfile('*.*');
fullfilename = fullfile(directory,im_file);

[x] = imread(fullfilename); %read in image, reports as 3 dim matrix in RGB format

a=x(:,:,1)==RGB(1); b=x(:,:,2)==RGB(2); c=x(:,:,3)==RGB(3); %finds pixels of a certain color

im=a.*b.*c; %creates a matrix of pixels representing the pixels of the color of interest

figure(1)
contour(im) %plot the reduced image vector

clear a b c name directory im_file

%% Convert the Lines to X,Y points
dat.xscale=dat.page_size(1)/length(im(1,:));
dat.yscale=dat.page_size(2)/length(im(:,1));

pts=zeros(sum(im(:)==1),2);
k=1;
for i=1:length(im(1,:))
    for j=1:length(im(:,1))
        
        if im(j,i)==1
            pts(k,1)=i*dat.xscale;
            pts(k,2)=j*dat.yscale;
            k=k+1;
            
        end
    end
end

figure(2)
plot(pts(:,1),pts(:,2),'*')

%% Reduce number of points

pts_new=reduce2D(pts,dat);
    
figure(3)
plot(pts_new(:,1),pts_new(:,2),'*')

%% Functions
function [ptsreduced]=reduce2D(pts,dat)
ptsreduced=reduce1D(pts,dat,dat.xscale,1);
ptsreduced=reduce1D(ptsreduced,dat,dat.yscale,2);

end

function [ptsreduced]=reduce1D(pts,dat,scale,c)

pts_new=zeros(size(pts));
j=1;
ptcount=zeros(dat.image_comp,2);
k=1;
point=ceil(dat.image_comp/2);

for i=1:length(pts(:,1))
     if (1<k) && (k<=dat.image_comp)
         if round(abs(abs(pts(i,c))-abs(pts(i-1,c))),5)<=round(scale,5)
             ptcount(k,1)=pts(i,1);
             ptcount(k,2)=pts(i,2);
             k=k+1;
                
             if k==dat.image_comp
                 pts_new(j,1)= ptcount(point,1);
                 pts_new(j,2)= ptcount(point,2);
                 j=j+1;
                 k=1;
             end
             
         else
             pts_new(j,1)= ptcount(ceil(k/2),1);
             pts_new(j,2)= ptcount(ceil(k/2),2);
             j=j+1;
             k=1;
         end
                        
    elseif k==1
        ptcount(k,1)=pts(i,1);
        ptcount(k,2)=pts(i,2);
        k=k+1;
    end

end

ptsreduced=pts_new(1:j-1,:);
end