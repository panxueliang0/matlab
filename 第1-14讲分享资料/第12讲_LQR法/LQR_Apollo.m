clc
clear
close all
load  path2.mat
%%
cf_ = 4e4;
cr_ = 6e4;
mass_=25000;
iz_=60000;
lf_ = 2.55;
lr_ = 2.675;
ts_ = 0.05;
lqr_eps_ = 0.01;
lqr_max_iteration_ = 150;
wheelbase_=5.225;

speed_actual=1;

matrix_a_ = [0,1,0,0;
             0,0,(cf_ + cr_) / mass_,0;
             0,0,0,1;
             (lf_ * cf_ - lr_ * cr_) / iz_,0,0,0;];
matrix_a_coeff_ = [0,0,0,0;
                   0,-(cf_ + cr_)/mass_,0,(lr_ * cr_ - lf_ * cf_) / mass_;
                   0,0,0,1;
                   0,(lr_ * cr_ - lf_ * cf_) / iz_,0,-1.0 * (lf_ * lf_ * cf_ + lr_ * lr_ * cr_) / iz_;];
matrix_b_ = [0;
             cf_ / mass_;
             0;
             lf_ * cf_ / iz_;];
matrix_bd_ = matrix_b_ * ts_;

matrix_state_ = zeros(4, 1);
matrix_k_ = zeros(1,4);
matrix_r_ = eye(1,1);
matrix_q_ = zeros(4,4);
matrix_q_(1, 1)=20;
matrix_q_(3, 3)=2; 
matrix_q_updated_ = matrix_q_;


