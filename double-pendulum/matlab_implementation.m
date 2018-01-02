function P1_double_pendulum
    clf; clear; close all;

%% Set up variables
    L1 = 1;      % Length (m)
    L2 = 1;      % Length (m)
    m1 = 2;      % Mass (Kg)
    m2 = 1;      % Mass (Kg)
    g = 9.8;     % Gravity acceleration (m/s^2)

    theta1_0 = pi/5;     % Initial y position (m)
    theta1_dot_0 = 0;    % Initial x position (m)
    theta2_0 = 5*pi/8;    % Initial y position (m)
    theta2_dot_0 = 0;    % Initial x position (m)

%% Run ODE
    initial = [theta1_0, theta1_dot_0, theta2_0, theta2_dot_0];
    t_span = 0:0.15:3.6;
    options = odeset('RelTol', 1e-5);
    [tout, zout] = ode45(@double_pendulum_ode, t_span, initial, options);

%% Plot position
    subplot(2, 2, 1)
    plot(tout, zout(:, 1));
    title('\theta_1 angle through time')
    xlabel('Time (s)')
    ylabel('radians')
   
    subplot(2, 2, 3)
    plot(tout, zout(:, 2));
    title('\theta_1 angular velocity through time')
    xlabel('Time (s)')
    ylabel('rad / s')

    subplot(2, 2, 2)
    plot(tout, zout(:, 3));
    title('\theta_2 angle through time')
    xlabel('Time (s)')
    ylabel('radians')

    subplot(2, 2, 4)
    plot(tout, zout(:, 4));
    title('\theta_2 angular velocity through time')
    xlabel('Time (s)')
    ylabel('rad / s')

%% Process for XYZ
    xy1 = zeros(length(tout), 2);  % (x, y) position (m)
    v1  = zeros(length(tout), 1);  % Speed w/o direction (m/2)
    xy2 = zeros(length(tout), 2);  % (x, y) position (m)
    v2  = zeros(length(tout), 1);  % Speed w/o direction (m/2)
    for i = 1:length(tout)
        xy1(i, 1) = -sin(zout(i, 1)) * L1;
        xy1(i, 2) = -cos(zout(i, 1)) * L1;
        tempx = -sin(zout(i, 3)) * L2;
        tempy = -cos(zout(i, 3)) * L2;
        xy2(i, 1) = xy1(i, 1) + tempx;
        xy2(i, 2) = xy1(i, 2) + tempy;
        v1(i) = -L1 * zout(i, 2);
        v2(i) = sqrt( (cos(zout(i, 3) - zout(i, 1))*v1(i) - L2 * zout(i, 4))^2 + (-sin(zout(i, 3) - zout(i, 1))*v1(i))^2 );
    end

%% Plot the animation
%     figure()
%     for i = 1:length(tout)
%         subplot(5, 5, i)
%         plot(xy1(i, 1), xy1(i, 2), '.', 'MarkerSize', 30)
%         hold on
%         plot([0, xy1(i, 1)], [0, xy1(i, 2)])
%         plot([xy1(i, 1), xy2(i, 1)], [xy1(i, 2), xy2(i, 2)])
%         plot(xy2(i, 1), xy2(i, 2), '.', 'MarkerSize', 20)
%         hold off
%         axis([-(L1+L2), L1+L2, -(L1+L2+0.75), (L1+L2-0.75)]*1.1);
%         xlabel('X (m)')
%         ylabel('Y (m)')
%         title( sprintf('Frame %d', i))
%         axis square;
%         pause(0.05)
%     end

%% Calculate energy
    KE1 = (1/2) * m1 * v1.^2;
    KE2 = (1/2) * m2 * v2.^2;
    PE1 = m1 * g * (xy1(:, 2) + L1);
    PE2 = m2 * g * (xy2(:, 2) + L1+L2);
    TotalE1 = KE1 + PE1;
    TotalE2 = KE2 + PE2;
    TotalE = TotalE1 + TotalE2;

%% Plot Energy
    figure()
    plot(tout, KE1, 'linewidth', 2);
    hold on
    plot(tout, KE2, 'r', 'linewidth', 2);
    plot(tout, PE1, 'g', 'linewidth', 2);
    plot(tout, PE2, 'm', 'linewidth', 2);
    plot(tout, TotalE, 'k', 'linewidth', 2);
    title('Energy in system over time')
    xlabel('Time (s)')
    ylabel('Energy (J)')
    legend('m_1 Kinetic', 'm_2 Kinetic', 'm_1 Potential', 'm_2 Potential', 'Total Energy')

%% Calculate acceleration and tension
    LHS_vec = zeros(length(tout), 4);
    for i = 1:length(tout)
        LHS_vec(i, :) = calculate_LHS_vec(zout(i, :));
    end

%% Plot acceleration and tension
    figure()
    subplot(2, 2, 1)
    plot(tout, LHS_vec(:, 1))
    title('Plot of \theta_1 acceleration')
    xlabel('Time (s)')
    ylabel('rad / s^2')
    
    subplot(2, 2, 2)
    plot(tout, LHS_vec(:, 2))
    title('Plot of \theta_2 acceleration')
    xlabel('Time (s)')
    ylabel('rad / s^2')

    subplot(2, 2, 3)
    plot(tout, LHS_vec(:, 3))
    title('Plot of F_{T1} (tension in first rod)')
    xlabel('Time (s)')
    ylabel('Force (N)')

    subplot(2, 2, 4)
    plot(tout, LHS_vec(:, 4))
    title('Plot of F_{T2} (tension in second rod)')
    xlabel('Time (s)')
    ylabel('Force (N)')


%% ODE function
    function zout = double_pendulum_ode(t, Z)
        ode_theta1_dot  = Z(2);
        ode_theta2_dot  = Z(4);
        out_vec = calculate_LHS_vec(Z);
        zout = [ode_theta1_dot; out_vec(1); ode_theta2_dot; out_vec(2)];
    end

    function lhs_vec = calculate_LHS_vec(Z)
        % Unpack variables
        ode_theta1      = Z(1);
        ode_theta1_dot  = Z(2);
        ode_theta2      = Z(3);
        ode_theta2_dot  = Z(4);

        % Make the M-Matrix
        M = [1, 0, 0, sin(ode_theta1 - ode_theta2)/(m1*L1);
             0, 0, -1, cos(ode_theta1 - ode_theta2);
             cos(ode_theta2 - ode_theta1), L2/L1, 0, 0;
             sin(ode_theta2 - ode_theta1), 0, 0, 1/(m2*L1)];

        % These 'elements' are those in the given RHS vector from the PSet
        el_1 = -(g/L1)*sin(ode_theta1);
        el_2 = -cos(ode_theta1)*m1*g - m1*L1*ode_theta1_dot^2;
        el_3 = -(g/L1)*sin(ode_theta2) - ode_theta1_dot^2*sin(ode_theta2 - ode_theta1);
        el_4 = (g/L1)*cos(ode_theta2) + (L2/L1)*ode_theta2_dot^2 + ode_theta1_dot^2*cos(ode_theta2 - ode_theta1);
        RHS_vec = [el_1; el_2; el_3; el_4];

        % Solve for the accelerations
        lhs_vec = M\RHS_vec;  % equivalent to inv(M) * RHS_vec
    end
end
