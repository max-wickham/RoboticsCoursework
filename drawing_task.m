% draw three lines and circle
controller = DrawingController();
controller.setup_controller();

% try
    controller.grab_pen();
    controller.draw_line([20,14], [20,6], 0, 1);
    controller.draw_line([20,6], [12.5,14], 1, 1);
    controller.draw_line([12.5,14], [20,14], 1, 1);
    controller.draw_circle_segment([20,10], 4, -pi/2, pi/2, 0, 0, 0);

% catch ME
%     ME
% 
% end 
controller.close_controller();
clear all;