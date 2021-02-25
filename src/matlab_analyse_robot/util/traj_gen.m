function [xef_d_0_t,vef_d_0_t,aef_d_0_t] = traj_gen(xi,xf,ti,tf,t)

    % constraints
    vi = [0,0,0];
    vf = [0,0,0];
    ai = [0,0,0];
    af = [0,0,0];

    x_constr = [xi(1) xf(1) vi(1) vf(1) ai(1) af(1)]';
    y_constr = [xi(2) xf(2) vi(2) vf(2) ai(2) af(2)]';
    z_constr = [xi(3) xf(3) vi(3) vf(3) ai(3) af(3)]';

    % system of equation with x = T*a;
    T = [1 ti ti^2 ti^3 ti^4 ti^5;
        1 tf tf^2 tf^3 tf^4 tf^5;
        0 1 2*ti 3*ti^2 4*ti^3 5*ti^4;
        0 1 2*tf 3*tf^2 4*tf^3 5*tf^4;
        0 0 2 6*ti 12*ti^2 20*ti^3;
        0 0 2 6*tf 12*tf^2 20*tf^3];

    % calculating coefficients
    % a = inv(T)*x but in matlab T\x is more efficient
    ax = T\x_constr;
    ay = T\y_constr;
    az = T\z_constr;

    % init desired values
    xef_d_0_t = zeros(3,1);
    vef_d_0_t = xef_d_0_t;
    aef_d_0_t = vef_d_0_t;

    % desired position at time t
    xef_d_0_t(1) = [1 t t^2 t^3 t^4 t^5] * ax;
    xef_d_0_t(2) = [1 t t^2 t^3 t^4 t^5] * ay;
    xef_d_0_t(3) = [1 t t^2 t^3 t^4 t^5] * az;
    % desired velocity at time t
    vef_d_0_t(1) = [0 1 2*t 3*t^2 4*t^3 5*t^4] * ax;
    vef_d_0_t(2) = [0 1 2*t 3*t^2 4*t^3 5*t^4] * ay;
    vef_d_0_t(3) = [0 1 2*t 3*t^2 4*t^3 5*t^4] * az; 
    % desired accelaration at time t
    aef_d_0_t(1) = [0 0 2 6*t 12*t^2 20*t^3] * ax;
    aef_d_0_t(2) = [0 0 2 6*t 12*t^2 20*t^3] * ay;
    aef_d_0_t(3) = [0 0 2 6*t 12*t^2 20*t^3] * az;     
    
end

