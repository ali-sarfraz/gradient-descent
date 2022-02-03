% Function used to compute Jacobian matrix for articulated manipulator.
    function [JVO1, JVO2, JVO3] = jacobian(theta_1,theta_2,theta_3)
    
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
    
% T matrix calculations.
    T1 = A1;
    T2 = T1*A2;
    T3 = T2*A3;
    
% For computing the Jacobian matrix, we need the following parameters.
% Z0, Z1, Z2, O0, O1, O2, O3.

% From their definitions, we know the following.
    O0 = [0;0;0];
    Z0 = [0;0;1];
    
% The Z vectors can be obtained from the 3rd column of the T matricies.
    Z1 = T1(1:3,3);
    Z2 = T2(1:3,3);
    
% The O vectors can be obtained from the 4th column of the T matricies.
    O1 = T1(1:3,4);
    O2 = T2(1:3,4);
    O3 = T3(1:3,4);
    
% Compute the individual JVO matricies for the RRR configuration frames.
    JVO1 = [cross(Z0, O1-O0), O0, O0];
    JVO2 = [cross(Z0, O2-O0), cross(Z1, O2-O1), O0];
    JVO3 = [cross(Z0, O3-O0), cross(Z1, O3-O1), cross(Z2, O3-O2)];
    
    end