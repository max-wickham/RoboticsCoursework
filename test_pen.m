controller = DrawingController();
controller.setup_controller();
% controller.move_servo(5,2000);
% % controller.move_servo(5,2430);
%%controller.set_speed_arm(1000,100);
%controller.set_speed_gripper();
try
%     controller.grab_pen();
%     controller.open_gripper();    
%     controller.close_gripper();
        %grab_pen();
    controller.draw_line([14,14], [18,18], 0, 0);
    controller.draw_circle_segment( [17,17], 3, 0, (2*pi-0.00001),0,0,1);
    controller.close_controller();

catch ME
    controller.close_controller();
    ME

end 
clear all;
%trajectory([10,10,10, pi/2],[10,0,10, pi/2])

% polar_to_cartesian([0,0.1,0.2], 5) + [0, 2]
