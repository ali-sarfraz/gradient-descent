    % Function for computing the repulsive forces for a given configuration.
    % Each origin gets its own eta value defined by the user.
    function [rep_force_1, rep_force_2, rep_force_3] = repulsive_force(current_q, eta_1, eta_2, eta_3)

    % Function for computing the repulsive force for a particular joint.
    % Approximates each obstacle as a line segment.
    function joint_rep_force = joint_repulsion(obstacle_start, obstacle_end, O_curr, obstacle_rho, obstacle_eta)
        distance = dot((obstacle_start - obstacle_end), (obstacle_end - O_curr)) / dot((obstacle_start - obstacle_end), (obstacle_start - obstacle_end));
        nearest_point = obstacle_end - distance * (obstacle_start - obstacle_end);
         
        % Compute distance between O and the closest point of the obstacle.
        rho_O = norm(O_curr - nearest_point);
         
        % Check if the manipulator is within the region of influence.
        if rho_O >= obstacle_rho
            % Repulsive force is going to be zero if outside region.
            joint_rep_force = [0;0;0]; 
        else
            % Compute the repulsive force using equation.
            gradient_ = (O_curr - nearest_point) / (norm(O_curr - nearest_point));
            joint_rep_force = obstacle_eta * ((1/rho_O) - (1/obstacle_rho)) * (1/rho_O^2) * gradient_;
        end
    end
    
    % Compute T-matrix for current configuration.
    [T01_curr, T02_curr, T03_curr] = t_matrix(current_q(1), current_q(2), current_q(3));
     
    % Compute the origin matricies for current configuration.
    O1_curr = T01_curr(1:3,4);
    O2_curr = T02_curr(1:3,4);
    O3_curr = T03_curr(1:3,4);

    %**************************************************************************
    %                   OBSTACLE ONE: SUPPORT COLUMN 1                        *
    %**************************************************************************
         
    % Assume that the column is modelled as a line segment.
    % Coordinates taken from base frame of manipulator in meters.
    % Region of influence taken from the centre of the column 1.
    column_1_start = [0; 1; 0];
    column_1_end = [0; 1; 2];
    rho_column_1 = 0.30;
    
    % Computations for overall joint repulsive forces.
    O1_rep_force_column_1 = joint_repulsion(column_1_start, column_1_end, O1_curr, rho_column_1, eta_1);
    O2_rep_force_column_1 = joint_repulsion(column_1_start, column_1_end, O2_curr, rho_column_1, eta_1);
    O3_rep_force_column_1 = joint_repulsion(column_1_start, column_1_end, O3_curr, rho_column_1, eta_1);

    %**************************************************************************
    %                   OBSTACLE TWO: SUPPORT COLUMN 2                        *
    %**************************************************************************
     
    % Assume that the column is modelled as a line segment.
    % Coordinates taken from base frame of manipulator in meters.
    % Region of influence taken from the centre of the column 2.
    column_2_start = [0; -1; 0];
    column_2_end = [0; -1; 2];
    rho_column_2 = 0.30;

    % Computations for overall joint repulsive forces.
    O1_rep_force_column_2 = joint_repulsion(column_2_start, column_2_end, O1_curr, rho_column_2, eta_2);
    O2_rep_force_column_2 = joint_repulsion(column_2_start, column_2_end, O2_curr, rho_column_2, eta_2);
    O3_rep_force_column_2 = joint_repulsion(column_2_start, column_2_end, O3_curr, rho_column_2, eta_2);
         
    %**************************************************************************
    %               OBSTACLE THREE: CLOSEST ASSEMBLY LINE CORNER              *
    %**************************************************************************
     
    % Assume that the top to the assembly line corner is modelled as a line segment.
    % Coordinates taken from base frame of manipulator in meters.
    % Region of influence taken from the centre of the assembly line.
    assem_start = [-0.8; -1; 0.7];
    assem_end = [-0.8; 1; 0.7];
    rho_assem = 0.20;

    % Computations for overall joint repulsive forces.
    O1_rep_force_assem = joint_repulsion(assem_start, assem_end, O1_curr, rho_assem, eta_3);
    O2_rep_force_assem = joint_repulsion(assem_start, assem_end, O2_curr, rho_assem, eta_3);
    O3_rep_force_assem = joint_repulsion(assem_start, assem_end, O3_curr, rho_assem, eta_3);     

    %**************************************************************************
    %                             TOTAL REPULSION                             *
    %**************************************************************************
     
    rep_force_1 = O1_rep_force_column_1 + O1_rep_force_column_2 + O1_rep_force_assem;
    rep_force_2 = O2_rep_force_column_1 + O2_rep_force_column_2 + O2_rep_force_assem;
    rep_force_3 = O3_rep_force_column_1 + O3_rep_force_column_2 + O3_rep_force_assem;
     
    end
    