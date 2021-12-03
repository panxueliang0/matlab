% LQR法
% 作者：PXL
% 日期：20210713
clc
clear
% close all
load  path2.mat

%% 相关参数定义
Kp=0.008;

dt = 0.1;
L = 1.73 ;
Q = [1   0   0   0  
     0   0   0   0  
     0   0   5   0  
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

matrix_size=4;
basic_state_size_=4;
matrix_a_=zeros(4,4);
matrix_a_(1, 2) = 1.0;
matrix_a_(2, 3) = (cf_ + cr_) / mass_;
matrix_a_(3, 4) = 1.0;
matrix_a_(4, 3) = (lf_ * cf_ - lr_ * cr_) / iz_;

matrix_a_coeff_ = zeros(matrix_size, matrix_size);
matrix_a_coeff_(2, 2) = -(cf_ + cr_) / mass_;
matrix_a_coeff_(2, 4) = (lr_ * cr_ - lf_ * cf_) / mass_;
matrix_a_coeff_(4, 2) = (lr_ * cr_ - lf_ * cf_) / iz_;
matrix_a_coeff_(4, 4) = -1.0 * (lf_ * lf_ * cf_ + lr_ * lr_ * cr_) / iz_;

matrix_b_ = zeros(basic_state_size_, 1);
matrix_bd_ = zeros(basic_state_size_, 1);
matrix_bdc_ = zeros(matrix_size, 1);
matrix_b_(2, 1) = cf_ / mass_;
matrix_b_(4, 1) = lf_ * cf_ / iz_;
matrix_bd_ = matrix_b_ * dt;

%% 轨迹处理
% 定义参考轨迹
refPos_x = path2(:,1);
refPos_y = path2(:,2);
refPos = [refPos_x, refPos_y];

% 计算航向角和曲率
diff_x = diff(refPos_x) ;
diff_x(end+1) = diff_x(end);
diff_y = diff(refPos_y) ;
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
x = refPos_x(1)+0.0; 
y = refPos_y(1)+1.0; 
yaw = refHeading(1)+0.0;
v = 0.01;
Delta = 0;
idx = 1;

% 轨迹跟踪实际量
pos_actual = [x,y];
v_actual  = v;
Delta_actual = Delta;
Delta_feedback=[];
idx_actual = 1;
latError_LQR = [];
pos_target = [refPos_x(1),refPos_y(1)];
refPos_Delta2=[];
contribution=[];
Error=[]
% 循环
while idx < length(refPos_x)-1
    % 寻找参考轨迹最近目标点
    idx = calc_target_index(x,y,refPos_x,refPos_y);  
    
    % LQR控制器
    [heading_error,lat_error, ...,
     lateral_contribution,lateral_rate_contribution,heading_contribution,heading_rate_contribution, ...,
     steer_angle_feedforward,delta,delta_r,latError] =   ...,
        LQR_control(idx,x,y,v,yaw,Delta,refPos_x,refPos_y,refHeading,refPos_Delta,refK,refSpeed,L,matrix_a_,matrix_a_coeff_,matrix_bd_,Q,R,dt);    
    
    % 如果误差过大，退出循迹
    if abs(latError) > 30
        disp('误差过大，退出程序!\n')
        break
    end
    
    % 计算加速度
    a = Kp* (refSpeed-v)/dt;
    
    % 更新状态
    [x,y,yaw,v,Delta] = update(a,x,y,yaw,v,Delta,delta, dt,L, refSpeed,delta_r);
    
    % 保存每一步的实际量
    pos_actual(end+1,:) = [x,y];
    pos_target(end+1,:) = [refPos_x(idx),refPos_y(idx)];
    v_actual(end+1,:)  = v;
    Delta_feedback(end+1)  = delta* 180 / pi /(6.2492 / pi * 180/10.85) * 100;
    Delta_actual(end+1)  = Delta;
    idx_actual(end+1) = idx;
    latError_LQR(end+1,:) =  [idx,latError];
    refPos_Delta2(end+1,:) =  steer_angle_feedforward;
    contribution(end+1,:)=[lateral_contribution,lateral_rate_contribution,heading_contribution,heading_rate_contribution];
    Error(end+1,:) = [heading_error,lat_error];
end

% 画图
figure(1)
clf;
axis equal;
axis on;
subplot(2,2,[1:2]);
plot(refPos_x,refPos_y,'r');
hold on;

for i = 1:20:size(pos_actual,1)-20
    subplot(2,2,[1:2]);
    scatter(pos_actual(i,1), pos_actual(i,2),150,'b.')
    scatter(pos_target(i+1,1), pos_target(i+1,2),150,'r.')
    axis([-20,100,-10,50]);
    pause(0.01);
    
    subplot(2,2,3);
    scatter(i, latError_LQR(i+1,2),150,'r.')
    hold on
    axis([0,size(pos_actual,1),-10,10]);
    
    subplot(2,2,4);
    scatter(i, Delta_actual(i+1),150,'r.')
    hold on
    axis([0,size(pos_actual,1),-0.3,0.3]);
end
figure(2)
clf;
subplot(2,1,1);
plot(Error);
hold on;
legend('heading_error','lat_error');

subplot(2,1,2);
plot(contribution);
hold on;
% plot(Delta_feedback);
% hold on;
legend('lateral_contribution','lateral_rate_contribution','heading_contribution','heading_rate_contribution');
% legend('Delta_feedback','r');
axis on;



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
function [heading_error,lat_error,lateral_contribution,lateral_rate_contribution,heading_contribution,heading_rate_contribution,steer_angle_feedforwardterm,Delta_delta,delta_r,latError] =  LQR_control(idx,x,y,v,yaw,delta,refPos_x,refPos_y,refPos_yaw,refPos_Delta,refPos_k,refSpeed,L,matrix_a_,matrix_a_coeff_,matrix_bd_,Q,R,dt)
% 求位置、航向角参考量
x_r = refPos_x(idx);
y_r = refPos_y(idx);
heading_r = refPos_yaw(idx);
delta_r = refPos_Delta(idx) ;

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
matrix_a_(2,2)=matrix_a_coeff_(2,2)/v;
matrix_a_(2,4)=matrix_a_coeff_(2,4)/v;
matrix_a_(4,2)=matrix_a_coeff_(4,2)/v;
matrix_a_(4,4)=matrix_a_coeff_(4,4)/v;
matrix_i = eye(4,4);
matrix_ad_ = inv(matrix_i - dt * 0.5 * matrix_a_)*(matrix_i + dt * 0.5 * matrix_a_);

K = calcu_K(matrix_ad_,matrix_bd_,Q,R);

% 获得速度误差量、前轮转角误差量两个控制量
u = -K * X;  % 2行1列
Delta_delta = u;

kv=0;
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
cf_=126050;
cr_=126050;
% steer_angle_feedforwardterm =(1.73 * refPos_k(idx) + kv * v * v * refPos_k(idx) - ..., 
%                  K(1, 3) *(lr_ * refPos_k(idx) -lf_ * mass_ * v * v * refPos_k(idx) / 2 / cr_ / 1.73))  ;
if idx>1000
    test=1;
end

kv = lr_ * mass_ / 2 / cf_ / wheelbase_ - lf_ * mass_ / 2 / cr_ / wheelbase_;
R2D=57.2957805;

% steer_angle_feedforwardterm =R2D*(wheelbase_ * refPos_k(idx)) - R2D*K(1, 3) *(lr_ * refPos_k(idx) -lf_ * mass_ * v * v * refPos_k(idx) / 2 / cr_ / wheelbase_);
steer_angle_feedforwardterm =R2D*(wheelbase_ * refPos_k(idx))
if refPos_k(idx) ~= 0
    test = R2D*K(1, 3) *(lr_ * refPos_k(idx) -lf_ * mass_ * v * v * refPos_k(idx) / 2 / cr_ / wheelbase_);
    steer_angle_feedforwardterm=steer_angle_feedforwardterm-test;
end

steer_single_direction_max_degree_ = 6.2492 / pi * 180/10.85;
lateral_contribution =-K(1, 1) * X(1, 1) * 180 / pi /steer_single_direction_max_degree_ * 100;
lateral_rate_contribution =-K(1, 2) * X(2, 1) * 180 / pi /steer_single_direction_max_degree_ * 100;
heading_contribution =-K(1, 3) * X(3, 1) * 180 / pi /steer_single_direction_max_degree_ * 100;
heading_rate_contribution =-K(1, 4) * X(4, 1) * 180 / pi /steer_single_direction_max_degree_ * 100;
heading_error = X(4,1);
lat_error = X(1,1);
end


%% 计算增益
function K = calcu_K (A,B,Q,R)

% 终止条件定义
iter_max = 500;
epsilon = 11;

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
K = (B' * P * B + R) \ (B' * P * A);  % 2行3列
% K = (B' * P * B + R)^-1 * (B' * P * A);  % 2行3列
% K = (B' * P * B + R)' *(B' * P * A);  % 2行3列
%   *ptr_K = (R + BT * P * B).inverse() * (BT * P * A + MT);

end

%% 更新状态
function [x, y, yaw, v, Delta] = update(a,x, y, yaw, v,Delta,Delta_delta,dt,L,refSpeed,refDelta)
% refDelta=0;
% if (refDelta + Delta_delta-Delta)>0.05
%     Delta = Delta+0.05;
% elseif (refDelta + Delta_delta-Delta)<-0.05
%     Delta = Delta-0.05;
% else
%     Delta = refDelta + Delta_delta;
% end
% 
% if Delta>0.5
%     Delta=0.5;
% elseif Delta<-0.5
%     Delta=-0.5;
% end
Delta = refDelta + Delta_delta;
% Delta = Delta_delta;
x = x + v * cos(yaw) * dt;
y = y + v * sin(yaw) * dt;
yaw = yaw + v / L * tan(Delta) * dt;
v = v + a*dt;
end
