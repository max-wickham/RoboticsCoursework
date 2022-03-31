% draw three lines and circle
controller = NewDrawingController();
controller.setup_controller();
controller.robotController.set_speed_gripper(1000);
controller.robotController.robot_model.kinematic_model.A3 = 11;
% try
     controller.grab_pen();
%     controller.robotController.set_arm_speed_mode(1500,100);
%     controller.draw_line([20+0.5,14+1], [20+0.5,6-0.3], 0, 1);
%     controller.draw_line([20+0.5,6-0.3], [12.5-0.5,14+1], 1, 1);
%     controller.draw_line([12.5-0.5,14+1], [20+0.5,14+1], 1, 1);
%     controller.draw_circle_segment([20+0.5,10+0.5], 4, pi/2-0.01, -pi/2+0.1, 1, 0, 1);
%     


    % controller.draw_line([20+0.8,14+1], [20+0.8,6+0.7], 0, 1);
    % controller.draw_line([20+0.8,6+0.7], [12.5,14+1], 1, 1);
    % controller.draw_line([12.5,14+1], [20+0.8,14+1], 1, 1);
    % controller.draw_circle_segment([20+0.5,10+1], 4, pi/2-0.01, -pi/2+0.23, 1, 0, 1);
    
    %controller.draw_circle_segment([15,10], 2, 0.01, -0.01, 0, 0, 0);
%     controller.draw_line([18,14], [14,18], 0, 0);

% catch ME
%     ME
% 
% end 
controller.close_controller();
clear all;