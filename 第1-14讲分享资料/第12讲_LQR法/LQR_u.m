% LQR法
% 作者：Ally
% 日期：20210429
clc
clear
close all
load  path2.mat

%% 相关参数定义
Kp=0.8;

dt = 0.1;
L = 1.73 ;
Q = 1*[1   0   0   0  
     0   0   0   0  
     0   0   0   0  
     0   0   0   0 ];
R = 1;

cf_=126050;
cr_=126050;

mass_fl=100;
mass_fr=100;
mass_rl=100;
mass_rr=100;
mass_front = mass_fl + mass_fr;
mass_rear = mass_rl + mass_rr;
mass_ = mass_front + mass_rear;
  
wheelbase_=1.73;
lf_ = wheelbase_ * (1.0 - mass_front / mass_);
lr_ = wheelbase_ * (1.0 - mass_rear / mass_);

iz_ = lf_ * lf_ * mass_front + lr_ * lr_ * mass_rear;
A=zeros(4,4);
A(1,2)=1.0;
A(2,2)=-(cf_+cr_)/mass_;
A(2,3)=(cf_+cr_)/mass_;
A(2,4)=(-cf_*lf_+cr_*lr_)/mass_;
A(3,4)=1.0;
A(4,2)=(-cf_*lf_+cr_*lr_)/iz_;
A(4,3)=(cf_*lf_-cr_*lr_)/iz_;
A(4,4)=(-cf_*lf_*lf_-cr_*lr_*lr_)/iz_;
B=zeros(4,1);
B(2,1)=cf_/mass_;
B(4,1)=cf_*lf_/iz_;

%% 轨迹处理
% 定义参考轨迹
refPos_x = path2(:,1);
refPos_y = path2(:,2);
refPos = [refPos_x, refPos_y];

% 计算航向角和曲率
diff_x = diff(refPos_x)/dt ;
diff_x(end+1) = diff_x(end);
diff_y = diff(refPos_y)/dt ;
diff_y(end+1) = diff_y(end);
% derivative1 = gradient(refPos_y)/dt ./ abs(diff_x);              % 一阶导数
% derivative2 = del2(refPos_y) ./ abs(diff_x);                  % 二阶导数
refHeading = atan2(diff_y , diff_x);                   % 航向角
% refK = abs(derivative2) ./ (1+derivative1.^2).^(3/2);  % 计算曲率
refK = path2(:,3);
% 根据阿克曼转向原理，计算参考前轮转角
refPos_Delta = atan(L*refK);

% 参考速度
refSpeed = 1;

%% 主程序
% 赋初值
x = refPos_x(1)+0; 
y = refPos_y(1)-0.5; 
yaw = refHeading(1)+0.0;
v = 1;
Delta = 0;
idx = 1;

% 轨迹跟踪实际量
pos_actual = [x,y];
v_actual  = v;
Delta_actual = Delta;
idx_actual = 1;
latError_LQR = [];

% 循环
while idx < length(refPos_x)-1
    % 寻找参考轨迹最近目标点
    idx = calc_target_index(x,y,refPos_x,refPos_y);  
%     idx = idx+1;
    % LQR控制器
    [delta,delta_r,latError] =  LQR_control(idx,x,y,v,yaw,Delta,refPos_x,refPos_y,refHeading,refPos_Delta,refK,refSpeed,L,A,B,Q,R,dt);    
    
    % 如果误差过大，退出循迹
    if abs(latError) > 10
        disp('误差过大，退出程序!\n')
        break
    end
    
    % 计算加速度
    a = Kp* (refSpeed-v)/dt;
    
    % 更新状态
    [x,y,yaw,v,Delta] = update(a,x,y,yaw,v,Delta,delta, dt,L, refSpeed,delta_r);
    
    % 保存每一步的实际量
    pos_actual(end+1,:) = [x,y];
    v_actual(end+1,:)  = v;
    Delta_actual(end+1)  = Delta;
    idx_actual(end+1) = idx;
    latError_LQR(end+1,:) =  [idx,latError];
end

% 画图
figure
plot(refPos_x,refPos_y,'r')
hold on
for i = 1:size(pos_actual,1)
    scatter(pos_actual(i,1), pos_actual(i,2),150,'b.')
    pause(0.01);
end

% 保存
path_LQR = pos_actual;
save path_LQR.mat path_LQR
save latError_LQR.mat latError_LQR

%% 寻找参考轨迹最近目标点
function target_idx = calc_target_index(pos_x,pos_y, refPos_x,refPos_y)
i = 1:length(refPos_x)-1;
dist = sqrt((refPos_x(i)-pos_x).^2 + (refPos_y(i)-pos_y).^2);
[~, target_idx] = min(dist);
end


%% LQR控制
function [Delta_delta,delta_r,latError] =  LQR_control(idx,x,y,v,yaw,delta,refPos_x,refPos_y,refPos_yaw,refPos_Delta,refPos_k,refSpeed,L,A,B,Q,R,dt)
% 求位置、航向角参考量
x_r = refPos_x(idx);
y_r = refPos_y(idx);
heading_r = refPos_yaw(idx);
delta_r = refPos_Delta(idx);

% 求位置、航向角的误差
x_error  = x - x_r;
y_error = y - y_r;
yaw_error =  yaw - heading_r;

% 根据百度Apolo，计算横向误差
latError = y_error*cos(heading_r) - x_error*sin(heading_r);

latError_dot=v*sin(yaw_error);

r=L/tan(delta);
heading_rate=v/r;
ref_heading_rate=refPos_k(idx)*refSpeed;

% 将误差值赋值到状态量
X(1,1) = latError; 
X(2,1) = latError_dot;  
X(3,1) = yaw_error;
X(4,1) = heading_rate-ref_heading_rate;

% 由状态方程矩阵系数，计算K
A(2,2)=A(2,2)/v;
A(2,4)=A(2,4)/v;
A(4,2)=A(4,2)/v;
A(4,4)=A(4,4)/v;

K = calcu_K(A,B,Q,R);

% 获得速度误差量、前轮转角误差量两个控制量
u = -K * X;  % 2行1列
Delta_delta = u;
end


%% 计算增益
function K = calcu_K (A,B,Q,R)

% 终止条件定义
iter_max = 500;
epsilon = 0.01;

% 循环
P_old = Q;
for i = 1:iter_max
    P_new = A' * P_old * A - (A' * P_old * B) / (R + B' * P_old * B) *( B' * P_old * A) +Q;
    if abs(P_new - P_old) <= epsilon
        break
    else
        P_old = P_new; 
    end
end

P = P_new;
K = (B' * P * B + R)^-1 * (B' * P * A);  % 2行3列
end

%% 更新状态
function [x, y, yaw, v, Delta] = update(a,x, y, yaw, v,Delta,Delta_delta,dt,L,refSpeed,refDelta)
% if (refDelta + Delta_delta-Delta)>0.1
%     Delta = Delta+0.1;
% elseif (refDelta + Delta_delta-Delta)<-0.1
%     Delta = Delta-0.1;
% else
%     Delta = refDelta + Delta_delta;
% end
% 
% if Delta>0.5
%     Delta=1.5;
% elseif Delta<-0.5
%     Delta=-1.5;
% end
Delta = refDelta + Delta_delta;
x = x + v * cos(yaw) * dt;
y = y + v * sin(yaw) * dt;
yaw = yaw + v / L * tan(Delta) * dt;
v = v + a*dt;
end
