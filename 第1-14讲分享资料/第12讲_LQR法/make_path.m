clc
clear
close all
%%

x=[0:0.1:80]
y=zeros(1,801)

p=[-pi/2:0.01:pi/2]
x1=cos(p)*10
y1=sin(p)*10+10
% plot(x1,y1)
x1=x1+x(end)
y1=y1+y(end)
plot(x1,y1)
k1=ones(1,315)*0.1

x2=[80:-0.1:0]
y2=ones(1,801)*20

p=[pi:0.01:2*pi]
x3=sin(p)*10;
y3=cos(p)*10+30
plot(x3,y3)

x4=[0:0.1:80]
y4=ones(1,801)*40

x5=[x(1:end-1) x1(1:end-1) x2(1:end-1) x3(1:end-1) x4]
y5=[y(1:end-1) y1(1:end-1) y2(1:end-1) y3(1:end-1) y4]
plot(x5,y5)
x=x5
y=y5
plot(x,y)

path2=[x' y']

k=zeros(1,801);
k1=ones(1,315)*0.1
k2=ones(1,801)*0
k3=ones(1,315)*-0.1
k4=ones(1,801)*0

k5=[k(1:end-1) k1(1:end-1) k2(1:end-1) k3(1:end-1) k4]
path2=[x' y' k5'];



