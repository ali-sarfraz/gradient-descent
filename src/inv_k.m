% Function used to compute configuration and joint variables for articulated manipulator
% by taking in the end effector position as an argument.
    function [q_1,q_2] = inverse_kin(end_x,end_y,end_z)

% Define link lengths as per the manipulator.
    L1 = 0.8; % meters
    L2 = 0.6; % meters
    L3 = 0.5; % meters
    
% Define equation for computing theta_1.
    theta_1 = atan2d(end_y,end_x);
% In the event that the links are vastly different in lengths, there is another
% potential solution for theta_1, which is an offset of 180 degrees.
% i.e theta_1b = theta_1 + PI.

% Define equation for computing theta_3.
    C3 = ((end_x^2) + (end_y^2) + (end_z - L1)^2 - L3^2 - L2^2)/(2*L3*L2);
    S3_positive = sqrt(1-((C3)^2));
    S3_negative = -S3_positive; % Account for the +/- value of S3.
    
    theta_3_positive = atan2d(S3_positive,C3);
    theta_3_negative = atan2d(S3_negative,C3);
    
% Define equation for computing theta_2, similar to in class example.
    alpha_angle = atan2d(end_z-L1,sqrt((end_x^2)+(end_y^2)));

% Calculation of the angle Beta depends on the value of S3, so we have two
% options once again.
    beta_angle_positive = atan2d(L3*S3_positive,L2+(L3*C3));
    beta_angle_negative = atan2d(L3*S3_negative,L2+(L3*C3));   
    
    % Upper shoulder.
    theta_2_positive = alpha_angle - beta_angle_positive;
    % Lower shoulder.
    theta_2_negative = alpha_angle - beta_angle_negative;


% Gather all angles into final two configurations.
    q_1 = [theta_1 theta_2_positive theta_3_positive];
    q_2 = [theta_1 theta_2_negative theta_3_negative];
    end