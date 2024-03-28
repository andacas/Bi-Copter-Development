% Call the functions sequentially
[xddot, yddot, phiddot] = calculate_EOM();
animate_bicopter();
animate_figure8();

% Define file1 function
function [xddot, yddot, phiddot] = calculate_EOM()
    syms x y phi xdot ydot phidot xddot yddot phiddot m I g l u1 u2 real;

    T = 0.5*m*(xdot^2 + ydot^2) + 0.5*I*phidot^2;
    V = m*g*y;
    L = T - V;

    f1 = [x + 0.5*l*cos(phi); y + 0.5*l*sin(phi)];
    z1 = [x; y; phi];
    J1 = jacobian(f1, z1);

    f2 = [x - 0.5*l*cos(phi); y - 0.5*l*sin(phi)];
    z2 = [x; y; phi];
    J2 = jacobian(f2, z2);

    F1 = [-u1*sin(phi); u1*cos(phi)];
    F2 = [-u2*sin(phi); u2*cos(phi)];

    F = simplify(transpose(J1)*F1 + transpose(J2)*F2);

    Fx = F(1);
    Fy = F(2);
    tau = F(3);

    dLdqdot = [diff(L, xdot); diff(L, ydot); diff(L, phidot)];
    ddt_dLdqdot = simplify(sum(jacobian(dLdqdot, [x; y; phi]) * [xddot; yddot; phiddot]) + ...
                           sum(jacobian(dLdqdot, [xdot; ydot; phidot]) * [diff(xddot, 't'); diff(yddot, 't'); diff(phiddot, 't')]));
    dLdq = [diff(L, x); diff(L, y); diff(L, phi)];

    EOM_x = ddt_dLdqdot(1) - dLdq(1) - Fx;
    EOM_y = ddt_dLdqdot(2) - dLdq(2) - Fy;
    EOM_phi = ddt_dLdqdot(3) - dLdq(3) - tau;

    xddot = solve(EOM_x, xddot);
    yddot = solve(EOM_y, yddot);
    phiddot = solve(EOM_phi, phiddot);
end



% Define file2 function
function animate_bicopter()
    % Import libraries
    addpath('path/to/required/libraries');
    % Define parameters
    parms.m = 1;
    parms.I = 0.1;
    parms.g = 9.81;
    parms.l = 0.2;
    parms.r = 0.05;
    parms.pause = 0.01;
    parms.fps = 30;
    % Define controller
    controller = @(x,y,phi,xdot,ydot,phidot,m,I,g,l) controller(x,y,phi,xdot,ydot,phidot,m,I,g,l);
    % Define bicopter_rhs
    bicopter_rhs = @(z,t,m,I,g,l) bicopter_rhs(z,t,m,I,g,l);
    % Call animate function
    animate(t,z,parms);
end

% Define file3 function
function animate_figure8()
    % Import libraries
    addpath('path/to/required/libraries');
    % Define parameters
    parms.m = 1;
    parms.I = 0.1;
    parms.g = 9.81;
    parms.l = 0.2;
    parms.r = 0.05;
    parms.pause = 0.01;
    parms.fps = 30;
    parms.kp_y = 300;
    parms.kp_x = 300;
    parms.kp_phi = 2500;
    parms.kd_phi = 2*sqrt(parms.kp_phi);
    % Define figure8 function
    [t, x_ref, y_ref, xdot_ref, ydot_ref, x2dot_ref, y2dot_ref] = figure8(x0_l, y0_l, h, t0, tN);
    % Define controller
    controller = @(x,y,phi,xdot,ydot,phidot,kp_y,kp_x,kp_phi,kd_phi,x_ref,y_ref,xdot_ref,ydot_ref,x2dot_ref,y2dot_ref,m,I,g,l) ...
                  controller(x,y,phi,xdot,ydot,phidot,kp_y,kp_x,kp_phi,kd_phi,x_ref,y_ref,xdot_ref,ydot_ref,x2dot_ref,y2dot_ref,m,I,g,l);
    % Define bicopter_rhs
    bicopter_rhs = @(z,t,m,I,g,l,kp_y,kp_x,kp_phi,kd_phi,x_ref,y_ref,xdot_ref,ydot_ref,x2dot_ref,y2dot_ref) ...
                    bicopter_rhs(z,t,m,I,g,l,kp_y,kp_x,kp_phi,kd_phi,x_ref,y_ref,xdot_ref,ydot_ref,x2dot_ref,y2dot_ref);
    % Call animate function
    animate(t,z,parms);
end