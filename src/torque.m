    % This function makes use of the attractive and repulsive potential fields
    % to generate the workspace forces. These forces are then converted into
    % torques using the Jacobian matrix script.
    function torque_vector = torque(current_q, ending_q, zeta_1, zeta_2, zeta_3, d_value, eta_1, eta_2, eta_3)
         
    % Determine attractive forces for current configuration.
    [att_force_1, att_force_2, att_force_3] = attractive_force(current_q, ending_q, zeta_1, zeta_2, zeta_3, d_value);
     
    % Determine repuslive forces for current configuration.
    [rep_force_1, rep_force_2, rep_force_3] = repulsive_force(current_q, eta_1, eta_2, eta_3);
     
    % Determine intermediate jacobian matricies.
    [JVO1, JVO2, JVO3] = jacobian(current_q(1), current_q(2), current_q(3));
     
    % Calculate attractive torques.
    att_torque_1 = transpose(JVO1) * att_force_1;
    att_torque_2 = transpose(JVO2) * att_force_2;
    att_torque_3 = transpose(JVO3) * att_force_3;
     
    % Calculate repulsive torques.
    rep_torque_1 = transpose(JVO1) * rep_force_1;
    rep_torque_2 = transpose(JVO2) * rep_force_2;
    rep_torque_3 = transpose(JVO3) * rep_force_3;
     
    % Calculate total torques.
    total_t1 = att_torque_1 + rep_torque_1;
    total_t2 = att_torque_2 + rep_torque_2;
    total_t3 = att_torque_3 + rep_torque_3;
    
    torque_vector = total_t1 + total_t2 + total_t3;
     
    end
    