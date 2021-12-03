clear all;
clc;
filename='/home/apollo/桌面/data1.xlsx'
num = xlsread(filename, 1);


% x1=437887.305696044;
% y1=4433040.36715134;
% x2=37.8263+437847-x1;
% y2=5.5697+4433039-y1;

% x1=437867.2010795890;
% y1=443303.8323491960;

x0=num(1,1);
y0=num(1,2);

x=num(:,1)-x0;
y=num(:,2)-y0;
x1=x(1);
y1=y(1);
x2=x(1200);
y2=y(1200);
xz=(x1+x2)/2.0;
yz=(y1+y2)/2.0;
L=sqrt((x1-x2)^2+(y1-y2)^2)
breadth=2;
theta=atan2(y2-y1,x2-x1);
xl=[x1:0.01:x2]-breadth/2.0*sin(theta);
yl=tan(theta)*(xl-x1)+y1+breadth/2.0/cos(theta);
xr=[x1:0.01:x2]+breadth/2.0*sin(theta)
yr=tan(theta)*(xr-x1)+y1-breadth/2.0/cos(theta);
figure(1)
clf;
plot(x-x1,y-y1)
hold on;
plot(xl-x1,yl-y1,'r')
hold on;
plot(xr-x1,yr-y1,'r')
hold on;
scatter(xz,yz)
hold on;
axis equal;


% x3=437879.722246044-437887.305696044;
% y3=4433046.98429134-4433040.36715134;
% x4=437875.67904539-437887.305696044;
% y4=4433046.58808531-4433040.36715134;
x3=x(2200);
y3=y(2200);
x4=x(3100);
y4=y(3100);

xz2=(x3+x4)/2.0;
yz2=(y3+y4)/2.0;
L2=sqrt((x3-x4)^2+(y3-y4)^2)
theta2=atan2(y4-y3,x4-x3);
xl2=[x3:-0.01:x4]-breadth/2.0*sin(theta2);
yl2=tan(theta2)*(xl2-x3)+y3+breadth/2.0/cos(theta2);
xr2=[x3:-0.01:x4]+breadth/2.0*sin(theta2)
yr2=tan(theta2)*(xr2-x3)+y3-breadth/2.0/cos(theta2);
figure(1)
plot(xl2,yl2,'r')
hold on;
plot(xr2,yr2,'r')
hold on;
scatter(xz2,yz2)
hold on;
axis equal;


x5=x(4200);
y5=y(4200);
x6=x(6000);
y6=y(6000);
xz3=(x5+x6)/2.0;
yz3=(y5+y6)/2.0;
L3=sqrt((x5-x6)^2+(y5-y6)^2)
theta3=atan2(y6-y5,x6-x5);
xl3=[x5:0.01:x6]-breadth/2.0*sin(theta3);
yl3=tan(theta3)*(xl3-x5)+y5+breadth/2.0/cos(theta3);
xr3=[x5:0.01:x6]+breadth/2.0*sin(theta3)
yr3=tan(theta3)*(xr3-x5)+y5-breadth/2.0/cos(theta3);
figure(1)
plot(xl3,yl3,'r')
hold on;
plot(xr3,yr3,'r')
hold on;
scatter(xz3,yz3)
hold on;
axis equal;
%%
for i=1:2459
    deltx(i)=x(i)-xz;
    delty(i)=y(i)-yz;
    deltx_(i)=deltx(i)*cos(theta)+delty(i)*sin(theta);
    delty_(i)=-deltx(i)*sin(theta)+delty(i)*cos(theta);
    if abs(deltx_(i))>L/2.0 || abs(delty_(i))>breadth/2.0
            behavior(i)=0;
    else
            behavior(i)=1;
    end
end

% 
% figure(2)
% clf;
% plot(behavior)
% hold on;

%%

[xc,yc,R,a] = circfit(x(1200:2200),y(1200:2200))

t = (-90:0.5:100)*pi/180;
xxl = (R-breadth/2.0)*cos(t)+xc;
yyl = (R-breadth/2.0)*sin(t)+yc;
xxr = (R+breadth/2.0)*cos(t)+xc;
yyr = (R+breadth/2.0)*sin(t)+yc;
figure(1)
plot(xxl,yyl,'k')
hold on;
plot(xxr,yyr,'k')
hold on;
grid on;

[xc2,yc2,R2,a2] = circfit(x(3000:4000),y(3000:4000))

t2 = (80:0.5:290)*pi/180;
xxl2 = (R2-breadth/2.0)*cos(t2)+xc2;
yyl2 = (R2-breadth/2.0)*sin(t2)+yc2;
xxr2 = (R2+breadth/2.0)*cos(t2)+xc2;
yyr2 = (R2+breadth/2.0)*sin(t2)+yc2;
figure(1)
plot(xxl2,yyl2,'k')
hold on;
plot(xxr2,yyr2,'k')
hold on;
grid on;
%%
function [xc,yc,R,a] = circfit(x,y)
%圆拟合函数
%CIRCFIT Fits a circle in x,y plane
% [XC, YC, R, A] = CIRCFIT(X,Y)
% Result is center point (yc,xc) and radius R.A is an
% optional output describing the circle’s equation:
%
% x^2+y^2+a(1)*x+a(2)*y+a(3)=0
% by Bucher izhak 25/oct/1991
n=length(x); xx=x.*x; yy=y.*y; xy=x.*y;
A=[sum(x) sum(y) n;sum(xy) sum(yy)...
sum(y);sum(xx) sum(xy) sum(x)];
B=[-sum(xx+yy) ; -sum(xx.*y+yy.*y) ; -sum(xx.*x+xy.*y)];
a=A\B;            %x = A\B 用来求解线性方程 A*x = B.  A 和 B 的行数一致.
xc = -.5*a(1);
yc = -.5*a(2);
R = sqrt((a(1)^2+a(2)^2)/4-a(3));
 theta=0:0.1:2*pi;  
    Circle1=xc+R*cos(theta);  
    Circle2=yc+R*sin(theta);   
%     plot(Circle1,Circle2,'g','linewidth',1);  
    axis equal  
end 

    