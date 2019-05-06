clear;
clc;
data=load('DirecAngle2.txt');
datax=load('DirecAngleCar22.txt');
figure(1);
hist(data(:,1),20);
 figure(2);
hist(data(:,2),20);
 figure(3);
 hist(data(:,3),20);
%  figure(4);
%  hist(datax(:,1),20);
% figure(5);
% hist(datax(:,2),20);
% figure(6);
% hist(datax(:,3),20);
[zNum,zAngleValue]=hist(data(:,1),20);
[xNum,xAngleValue]=hist(data(:,2),20);
[yNum,yAngleValue]=hist(data(:,3),20);
% [zNumx,zAngleValuex]=hist(datax(:,1),20);
% [xNumx,xAngleValuex]=hist(datax(:,2),20);
% [yNumx,yAngleValuex]=hist(datax(:,3),20);
% 
 z1=zAngleValue(find(zNum==(max(zNum))));
% z2=zAngleValuex(find(zNumx==(max(zNumx))));
 x1=xAngleValue(find(xNum==(max(xNum))));
% x2=xAngleValuex(find(xNumx==(max(xNumx))));
 y1=yAngleValue(find(yNum==(max(yNum))));
% y2=yAngleValuex(find(yNumx==(max(yNumx))));
% 
% if(z2>z1)   
%     zx=z2-z1;
% else
%     zx=2*pi-(z1-z2);
% end
% 
% if(y2>y1)
%     yx=y2-y1;
% else
%     yx=2*pi-(y1-y2);
% end
% 
% if(x2>x1)
%     xx=x2-x1;
% else
%     xx=2*pi-(x1-x2);
% end
% 
% R_rotm=eul2rotm([zx yx xx],'ZYX')
% R_rotm2=angle2dcm(zx,yx,xx,'ZYX')