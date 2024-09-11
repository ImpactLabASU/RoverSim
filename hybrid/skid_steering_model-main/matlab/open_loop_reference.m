function [Omega_r,Omega_l,global_turn_right,auto_drive,check_position,update_compass,update_gps,continue_moving,omega,init_position_x, init_position_y] = open_loop_reference(t,a,b,r,states,global_turn_right,auto_drive,check_position,omega,update_compass,update_gps,continue_moving,init_position_x, init_position_y)

    %Provide an open loop command to the wheels rotation speeds
   % Omega_r = 1.55;
   % Omega_l = -1.55/-2;

    %Use this to set v_x and omega_z based on the kinematic model
    M = [0.5, 0.5; b/(2*(a^2+b^2)), -b/(2*(a^2+b^2))];
    %v = computeNewVelocity();
    [v,omega,global_turn_right,auto_drive,check_position,update_compass,update_gps,continue_moving,init_position_x, init_position_y] = computeNewAngleAndVel(t,states,global_turn_right,auto_drive,check_position,omega,update_compass,update_gps,continue_moving,init_position_x, init_position_y);%0.5*cos(t).^2+0.5*sin(t).^2;
    Vde = M^(-1)*[v; omega];
    Omega_r = Vde(1)/r;
    Omega_l = Vde(2)/r;
 
end


function [v,omega,global_turn_right,auto_drive,check_position,update_compass,update_gps,continue_moving,init_position_x, init_position_y] =computeNewAngleAndVel(t,states,global_turn_right,auto_drive,check_position,omega,update_compass,update_gps,continue_moving,init_position_x, init_position_y)
    v = 0.5;
    %init_position_x = 0;
    %init_position_y = 0;
    cur_position_x = states(1);
    cur_position_y = states(2);
    thresh = 1;
    prev_angle = omega;
    if(auto_drive && check_position)
        dist = ((init_position_y-cur_position_y)^2 + (init_position_x-cur_position_x)^2)^0.5;
        if(dist < thresh)
            v = 0.5;
            omega = 0;
            return;
        
        end
        
        check_position = 0;

        if(global_turn_right)
            turn_angle = 70;
        else
            turn_angle = -70;
        end

        omega = prev_angle + turn_angle;
        update_compass = 1;

    elseif(auto_drive && ~check_position && update_compass)
        cur_angle = omega;
        if((90 - cur_angle) <= 30)
            omega = -1000;
            update_compass = 0;
            update_gps = 1;
        end
     elseif(auto_drive && ~check_position && ~update_compass && update_gps)
        init_position_x = states(1);
        init_position_y = states(2);
        update_gps = 0;
        continue_moving = 1;
     elseif(auto_drive && ~check_position && ~update_compass && ~update_gps && continue_moving)
        omega = prev_angle;
        dist = ((init_position_y-cur_position_y)^2 + (init_position_x-cur_position_x)^2)^0.5;
        if(dist < thresh)
            v = 0.5;
            omega = 0;
            return;
        
        else
            v = 0;
            omega = 0;
        end

     end
end