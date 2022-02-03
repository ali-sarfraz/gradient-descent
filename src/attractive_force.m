    % Function for computing the attractive forces for a given configuration.
    function [att_force_1, att_force_2, att_force_3] = attractive_force(current_q, ending_q, zeta_1, zeta_2, zeta_3, d_value)
     
    % Compute T-matrix for current configuration.
    [T01_curr, T02_curr, T03_curr] = t_matrix(current_q(1), current_q(2), current_q(3));
    
    % Compute the origin matricies for current configuration.
    O1_curr = T01_curr(1:3,4);
    O2_curr = T02_curr(1:3,4);
    O3_curr = T03_curr(1:3,4);
    O_matrix_curr = [O1_curr,O2_curr,O3_curr];
    
    % Compute T-matrix for ending configuration.
    [T01_end, T02_end, T03_end] = t_matrix(ending_q(1), ending_q(2), ending_q(3));
        
    % Compute the origin matricies for final configuration.
    O1_end = T01_end(1:3,4);
    O2_end = T02_end(1:3,4);
    O3_end = T03_end(1:3,4);
    O_matrix_end = [O1_end, O2_end, O3_end];

    % Start with an empty matrix and append as we go.
    att_force_matrix = [];
     
    % Compute attractive forces for all 3 frames.
    for i = 1:3
        if i == 1
            zeta = zeta_1;
        elseif i == 2
            zeta = zeta_2;
        else
            zeta = zeta_3;
        end
        
        distance = norm(O_matrix_curr(:,i) - O_matrix_end(:,i));

        % Use conic potential field if we are far.
        if distance >= d_value
            force = -d_value * zeta * ( (O_matrix_curr(:,i) - O_matrix_end(:,i)) / distance);

        % Otherwise use parabolic potential field if we are close.
        else
            force = -zeta * (O_matrix_curr(:,i) - O_matrix_end(:,i));
        end

        att_force_matrix = [att_force_matrix, force];
      
    end

    att_force_1 = att_force_matrix(:,1);
    att_force_2 = att_force_matrix(:,2);
    att_force_3 = att_force_matrix(:,3);
     
    end
    