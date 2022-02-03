% Input the starting configuration.
q_start = [0, 0, 0];
  
% Input the goal position in the workspace.
x_pos = -0.9;
y_pos = 0.1;
z_pos = 1;
 
% Determine goal configuration using inverse kinematics.
[q_1, q_2] = inv_k(x_pos, y_pos, z_pos);
 
% As mentioned in previous report, only positive angles will
% be used in this project. Hence only q_1 is relevant.
q_end = q_1;
 
% Input torque calculation parameters.
current_q = q_start;
ending_q = q_end;       
zeta_1 = 3;      
zeta_2 = 2;
zeta_3 = 2;
eta_1 = 1;
eta_2 = 1;
eta_3 = 1;
d_value = 0.1; % meters.
 
% Input Gradient Descent parameters.
alpha_1 = 1;
alpha_2 = 1;
alpha_3 = 1;
epsilon = 1; 
epsilon_m = 1;

% Joint limits for each of the three DOF.
joint_1_max = 180;
joint_1_min = -180;
joint_2_max = 180;
joint_2_min = -180;
joint_3_max = 180;
joint_3_min = -180;

% End effector positions for mapping purposes.
ee_x = [];
ee_y = [];
ee_z = [];

% Begin Gradient Descent Algorithm.
for i = 1:1000
    % Calculate the unit torque vector using the torque generation script.
    torq_vector = torque(current_q, ending_q, zeta_1, zeta_2, zeta_3, d_value, eta_1, eta_2, eta_3);
    unit_torq_vector = torq_vector / (norm(torq_vector));
   
    % Ensure that we arent in a local minima by holding on to the previous configuration.
    previous_q = current_q;
    
    % Calculate the current x and y end effector positions.
    [to1, to2, to3] = t_matrix(current_q(1), current_q(2), current_q(3));
    ee_pos = to3(1:3,4);
    ee_x = [ee_x; ee_pos(1)];
    ee_y = [ee_y; ee_pos(2)];
    ee_z = [ee_z; ee_pos(3)];
    
    hold on
    
    % Plot the end effector position, along with obctacles in XY-Plane.
    plot(ee_x,ee_y,'g--o')  
    plot(0,1,'b--x')                    % Support column-1.
    plot(0,-1,'b--x')                   % Support column-2.
    plot([-0.8, -0.8], [-1,1], 'b--x')  % Assembly line.
    plot(0,0,'r--x')                    % Robot base.
    
    title ('End Effector Path in XY Plane')
    xlabel('X Position (m)')
    ylabel('Y Position (m)')
    
    % Plot the end effector position, along with obctacles in YZ-Plane.
    %    plot(ee_y,ee_z,'g--o')  
    %    plot([1,1],[0,2],'b--x')                    % Support column-1.
    %    plot([-1,-1],[0,2],'b--x')                  % Support column-2.
    %    plot(0,0,'r--x')                            % Robot base.
       
    %    title ('End Effector Path in YZ Plane')
    %    xlabel('Y Position (m)')
    %    ylabel('Z Position (m)')
    
    
    % Check if the goal configuration is clode enough to our current position.
    if norm(current_q - q_end) < epsilon
        % If we are within the right range then exit.
        break
        
    else
        % Compute the step for each joint.
        step =  [alpha_1 * unit_torq_vector(1), alpha_2 * unit_torq_vector(2), alpha_3 * unit_torq_vector(3)];
        
        % Ensure step doesnt exceed joint limits for first DOF.
        % Cap the values at max and min joint limits.
        if current_q(1) + step(1) > joint_1_max
            current_q = [joint_1_max, current_q(2), current_q(3)];
        elseif current_q(1) + step(1) < joint_1_min
            current_q = [joint_1_min, current_q(2), current_q(3)];
        else
            current_q = [current_q(1) + step(1), current_q(2), current_q(3)];
        end
        
        % Ensure step doesnt exceed joint limits for second DOF.
        % Cap the values at max and min joint limits.
        if current_q(2) + step(2) > joint_2_max
            current_q = [current_q(1), joint_2_max, current_q(3)];
        elseif current_q(2) + step(2) < joint_2_min
            current_q = [current_q(1), joint_2_min, current_q(3)];
        else
            current_q = [current_q(1), current_q(2) + step(2), current_q(3)];
        end
        
        % Ensure step doesnt exceed joint limits for third DOF.
        % Cap the values at max and min joint limits.
        if current_q(3) + step(3) > joint_3_max
            current_q = [current_q(1), current_q(2), joint_3_max];
        elseif current_q(3) + step(3) < joint_3_min
            current_q = [current_q(1), current_q(2), joint_3_min];
        else
            current_q = [current_q(1), current_q(2), current_q(3) + step(3)];
        end
        
    end
    
    % Employ random excitation algorithm if we are stuck in local minima.
    if i > 1 && (norm(current_q - previous_q) < epsilon_m)
        current_q = random_walk(current_q, unit_torq_vector, joint_1_max, joint_1_min, joint_2_max, joint_2_min, joint_3_max, joint_3_min);
    end
    
end

% Function to depoly a random walk if path gets stuck in a local minima.
function end_position = random_walk(current_q, unit_torq_vector, joint_1_max, joint_1_min, joint_2_max, joint_2_min, joint_3_max, joint_3_min)
    excitation_const = 4 * (rand() - 0.25);
    rand_step = transpose(excitation_const * unit_torq_vector);
   
    % Ensure rand_step doesnt exceed joint limits for first DOF.
    if (current_q(1) + rand_step(1)) > joint_1_max || (current_q(1) + rand_step(1)) < joint_1_min
        end_position = [current_q(1) - rand_step(1), current_q(2), current_q(3)];
    elseif (current_q(1) - rand_step(1)) > joint_1_max || (current_q(1) - rand_step(1)) < joint_1_min
        end_position = [current_q(1) + rand_step(1), current_q(2), current_q(3)];
    else
        end_position = [current_q(1) - rand_step(1), current_q(2),current_q(3)];
    end
    
    % Ensure rand_step doesnt exceed joint limits for second DOF.
    if (current_q(2) + rand_step(2)) > joint_2_max || (current_q(2) + rand_step(2)) < joint_2_min
        end_position = [current_q(1), current_q(2) - rand_step(2), current_q(3)];
    elseif (current_q(2) - rand_step(2)) > joint_2_max || (current_q(2) - rand_step(2)) < joint_2_min
        end_position = [current_q(1), current_q(2) + rand_step(2), current_q(3)];
    else
        end_position = [current_q(1), current_q(2) - rand_step(2), current_q(3)];
    end
    
    % Ensure rand_step doesnt exceed joint limits for third DOF.
    if (current_q(3) + rand_step(3)) > joint_3_max || (current_q(3) + rand_step(3)) < joint_3_min
        end_position = [current_q(1), current_q(2), current_q(3) - rand_step(3)];
    elseif (current_q(3) - rand_step(3)) > joint_3_max || (current_q(3) - rand_step(3)) < joint_3_min
        end_position = [current_q(1), current_q(2), current_q(3) + rand_step(3)];
    else
        end_position = [current_q(1), current_q(2), current_q(3) - rand_step(3)];
    end   
end