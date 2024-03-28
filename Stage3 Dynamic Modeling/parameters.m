function animate(t,z,parms)
    % Interpolation
    t_interp = linspace(t(1), t(end), parms.fps*(t(end)-t(1)));
    z_interp = interp1(t, z, t_interp);
    
    l = parms.l;
    r = parms.r;
    xxyy = max(max(abs(z_interp(:,1:2))));
    
    for i = 1:length(t_interp)
        x = z_interp(i,1);
        y = z_interp(i,2);
        phi = z_interp(i,3);

        R = [cos(phi), -sin(phi); sin(phi), cos(phi)];
        middle = [x; y];

        drone_left = middle + R * [-0.5*l; 0];
        axle_left = middle + R * [-0.5*l; 0.1];
        prop_left1 = middle + R * [-0.5*l + 0.5*r; 0.05];
        prop_left2 = middle + R * [-0.5*l - 0.5*r; 0.05];

        drone_right = middle + R * [0.5*l; 0];
        axle_right = middle + R * [0.5*l; 0.1];
        prop_right1 = middle + R * [0.5*l + 0.5*r; 0.05];
        prop_right2 = middle + R * [0.5*l - 0.5*r; 0.05];

        plot([drone_left(1), drone_right(1)], [drone_left(2), drone_right(2)], 'r', 'LineWidth', 5);
        hold on;
        plot([drone_left(1), axle_left(1)], [drone_left(2), axle_left(2)], 'g', 'LineWidth', 5);
        plot([prop_left1(1), prop_left2(1)], [prop_left1(2), prop_left2(2)], 'b', 'LineWidth', 5);
        plot([drone_right(1), axle_right(1)], [drone_right(2), axle_right(2)], 'g', 'LineWidth', 5);
        plot([prop_right1(1), prop_right2(1)], [prop_right1(2), prop_right2(2)], 'b', 'LineWidth', 5);
        plot(x, y, 'ko', 'MarkerSize', 2);
        hold off;

        axis([-xxyy-0.1, xxyy+0.1, -xxyy-0.1, xxyy+0.1]);
        axis equal;
        pause(parms.pause);
    end
end

function [t, x_ref, y_ref, xdot_ref, ydot_ref, x2dot_ref, y2dot_ref] = figure8(x0, y0, amp, T)
    t = linspace(0, T, 1000);
    x_ref = x0 + amp * cos(2*pi*t/T);
    y_ref = y0 + amp * sin(4*pi*t/T);
    xdot_ref = -2*pi*amp/T * sin(2*pi*t/T);
    ydot_ref = 4*pi*amp/T * cos(4*pi*t/T);
    x2dot_ref = -(2*pi*amp/T)^2 * cos(2*pi*t/T);
    y2dot_ref = -(4*pi*amp/T)^2 * sin(4*pi*t/T);
end

% File 3: Controller and dynamics

function [us, ud] = controller(x, y, phi, xdot, ydot, phidot, parms)
    us = parms.m * (parms.g + y2dot_ref + parms.kp_y * (y_ref - y) + parms.kd_y * (ydot_ref - ydot));
    phi_ref = -(1/parms.g) * (x2dot_ref + parms.kp_x * (x_ref - x) + parms.kd_x * (xdot_ref - xdot));
    ud = parms.kp_phi * (phi_ref - phi) + parms.kd_phi * (-phidot);
end

function zdot = bicopter_rhs(t, z, parms)
    x = z(1);
    y = z(2);
    phi = z(3);
    xdot = z(4);
    ydot = z(5);
    phidot = z(6);

    [us, ud] = controller(x, y, phi, xdot, ydot, phidot, parms);

    xddot = -(us / parms.m) * sin(phi);
    yddot = (us / parms.m) * cos(phi) - parms.g;
    phiddot = 0.5 * parms.l * ud / parms.I;

    zdot = [xdot; ydot; phidot; xddot; yddot; phiddot];
end