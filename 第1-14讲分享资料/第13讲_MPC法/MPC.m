% ����MPC���ٹ켣
% ���ߣ�Ally
% ���ڣ�2021/04/29
clc
clear
close all
load path2.mat

%% ��ʼ����
dt = 0.1;   % ʱ�䲽��
L = 1.73;    % ���
max_steer =33 * pi/180; % in rad
target_v =1;

%% �ο��켣����ز���
% ����ο��켣
refPos = path2;
refPos_x = refPos(:,1);
refPos_y = refPos(:,2);

% ���㺽��Ǻ�����
diff_x = diff(refPos_x) ;
diff_x(end+1) = diff_x(end);
diff_y = diff(refPos_y);
diff_y(end+1) = diff_y(end);
% derivative1 = gradient(refPos_y) ./ abs(diff_x);              % һ�׵���
% derivative2 = del2(refPos_y) ./ abs(diff_x);                  % ���׵���
refHeading = atan2(diff_y , diff_x);                   % �����
% refK = abs(derivative2) ./ (1+derivative1.^2).^(3/2);  % ��������
refK = path2(:,3);

% ���ݰ�����ת��ԭ������ο�ǰ��ת��
refDelta = atan(L*refK);

% % ��ͼ
% figure
% plot(refPos_x,refPos_y,'r-');
% hold on

%% ������
x = refPos_x(1)+2; 
y = refPos_y(1) + 2; 
yaw = refHeading(1)+0.2; 
v = 0.1;
U = [0.01;0.01];
idx =0;
pos_actual = [refPos_x,refPos_y];
latError_MPC = [];
pos_target = [refPos_x(1),refPos_y(1)];
Delta_actual = 0;

% ѭ��
while idx < length(refPos_x)-1
    
    % ����MPC������
    [Delta,v,idx,latError,U] = mpc_control(x,y,yaw,refPos_x,refPos_y,refHeading,refDelta,dt,L,U,target_v) ;
    
    % ���̫���˳�����
    if abs(latError) > 3
        disp('�������˳�����!\n')
        break
    end
    
    % ����״̬��
    [x,y,yaw] = updateState(x,y,yaw,v , Delta, dt,L, max_steer); 
    
    % ����ÿһ����ʵ����
    pos_actual(end+1,:) = [x,y];
    pos_target(end+1,:) = [refPos_x(idx),refPos_y(idx)];
    latError_MPC(end+1,:) = [idx,latError];
    Delta_actual(end+1)  = Delta;
    % �����ٹ켣ͼ
%     scatter(x,y,150,'b.');
%     pause(0.01);
end
%%
% ��ͼ
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
    scatter(i, latError_MPC(i+1,2),150,'r.')
    hold on
    axis([0,size(pos_actual,1),-10,10]);
    
    subplot(2,2,4);
    scatter(i, Delta_actual(i+1),150,'r.')
    hold on
    axis([0,size(pos_actual,1),-0.3,0.3]);
end
%% ����
path_MPC = pos_actual;
save path_MPC.mat path_MPC
save latError_MPC.mat latError_MPC