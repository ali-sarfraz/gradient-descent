% Function used to compute transformation matrix for articulated manipulator.
    function [TO1, TO2, TO3] = t_matrix(theta_1,theta_2,theta_3)

% Function used to compute the joint's A matrix.
    function a_matrix_ = a_matrix(a,d,alpha,theta)
        
    a_matrix_ = [cosd(theta),-sind(theta)*cosd(alpha),sind(theta)*sind(alpha),a*cosd(theta);
    sind(theta),cosd(theta)*cosd(alpha),-cosd(theta)*sind(alpha),a*sind(theta);
    0,sind(alpha),cosd(alpha),d;
    0,0,0,1];
    end

% Define variables and assign DH parameters as needed.
    L1 = 0.8; % meters
    L2 = 0.6; % meters
    L3 = 0.5; % meters
   
    a1 = 0;
    a2 = L2;
    a3 = L3;
    
    d1 = L1;
    d2 = 0;
    d3 = 0;
    
    alpha_1 = 90; % Degrees
    alpha_2 = 0;  % Degrees
    alpha_3 = 0;  % Degrees

    t1 = theta_1; % Joint Variable
    t2 = theta_2; % Joint Variable
    t3 = theta_3; % Joint Variable
    
% A matrix calculations.
    A1 = a_matrix(a1,d1,alpha_1,t1);
    A2 = a_matrix(a2,d2,alpha_2,t2);
    A3 = a_matrix(a3,d3,alpha_3,t3);
    
% Final T matrix calculations.
    TO1 = A1;
    TO2 = A1*A2;
    TO3 = A1*A2*A3;
    
    end