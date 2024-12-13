function [A_jacob, B_jacob] = get_A_B(desired_x, desired_u, params, dt)

    x = desired_x(1);
    y = desired_x(2);
    z = desired_x(3);
    xdt = desired_x(4);
    ydt = desired_x(5);
    zdt = desired_x(6);
    phi = desired_x(7);
    theta = desired_x(8);
    psi = desired_x(9);
    p = desired_x(10);
    q = desired_x(11);
    r = desired_x(12);

    u1 = desired_u(1);
    u2 = desired_u(2);
    u3 = desired_u(3);
    u4 = desired_u(4);

    m = params.mass;
    g = params.grav;
    Ixx = params.I(1,1);
    Iyy = params.I(2,2);
    Izz = params.I(3,3);

    transform = [cos(theta) 0 -cos(phi)*sin(theta);
                      0 1 0;
             sin(theta) 0 cos(phi)*cos(theta)];

    ang_rates = inv(transform) * [p; q; r];
    
    %% State variables
    state_var = [x; y; z; xdt; ydt; zdt; phi; theta; psi; p; q; r];
    input_var = [u1; u2; u3; u4;];
    
    f = [xdt;
         ydt;
         zdt;
         g*(theta * cos(psi) + phi * sin(psi));
         g*(theta * sin(psi) - phi * cos(psi));
         1/m * u1 - g;
         ang_rates(1);
         ang_rates(2);
         ang_rates(3);
         u2/Ixx;
         u3/Iyy;
         u4/Izz;];
    
    x_func = x + f*dt;
    
    %% State Equation
    
    A_jacob = jacobian(x_func, state_var);
    B_jacob = jacobian(x_func, input_var);
end