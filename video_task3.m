% draw three lines and circle
controller = NewDrawingController();
controller.setup_controller();
controller.robotController.set_speed_gripper(200);
controller.robotController.robot_model.kinematic_model.A3 = 11;
% try
    controller.grab_pen();
    controller.draw_line([20,14], [20,6], 0, 1);
    controller.draw_line([20,6], [12.5,14], 1, 1);
    controller.draw_line([12.5,14], [20,14], 1, 1);
    controller.draw_circle_segment([20,10], 4, pi/2-0.01, -pi/2+0.1, 1, 0, 1);

% catch ME
%     ME
% 
% end 
controller.close_controller();
clear all;