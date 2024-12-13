    params = crazyflie();
    syms x_ y_ z_ xdt_ ydt_ zdt_ phi_ theta_ psi_ p_ q_ r_ u1_ u2_ u3_ u4_ dt
    
    %% State variables
    state_var = [x_; y_; z_; xdt_; ydt_; zdt_; phi_; theta_; psi_; p_; q_; r_];
    state_input = [u1_; u2_; u3_; u4_];

    %% Crazyflie Params
    m   = params.mass;
    g   = params.grav;
    Ixx = params.I(1,1);
    Iyy = params.I(2,2);
    Izz = params.I(3,3);

    transform = [cos(theta_) 0 -cos(phi_)*sin(theta_);
                           0 1              sin(phi_);
                 sin(theta_) 0  cos(phi_)*cos(theta_) ];

    ang_rates = inv(transform) * [p_; q_; r_];
    
    f = [xdt_;
         ydt_;
         zdt_;
         g*(theta_ * cos(psi_) + phi_ * sin(psi_));
         g*(theta_ * sin(psi_) - phi_ * cos(psi_));
         (1/m * u1_) - g;
         ang_rates(1);
         ang_rates(2);
         ang_rates(3);
         u2_/Ixx;
         u3_/Iyy;
         u4_/Izz;];
    
    x_func = state_var + f*dt;
    
    %% State Equation
    A_jacob = jacobian(x_func, state_var);
    B_jacob = jacobian(x_func, state_input);

    % A = double(subs(A_jacob, state_var_sym, desired_x));
    % B = double(subs(B_jacob, state_input_sym, desired_u));