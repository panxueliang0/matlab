basic_state_size_=4;
preview_window_=1;
ts_=0.02;

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
 
matrix_size = basic_state_size_ + preview_window_;
matrix_a_ = zeros(basic_state_size_, basic_state_size_);
matrix_ad_ = zeros(basic_state_size_, basic_state_size_);
matrix_adc_ = zeros(matrix_size, matrix_size);

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
matrix_bd_ = matrix_b_ * ts_;

matrix_state_ = zeros(matrix_size, 1);
matrix_k_ = zeros(1, matrix_size);
matrix_r_ = ones(1, 1);
matrix_q_ = zeros(matrix_size, matrix_size);

matrix_q_ = [0.1   0   0   0   0
             0   0   0   0   0
             0   0   1   0   0
             0   0   0   0.2   0
             0   0   0   0   0]
             
             