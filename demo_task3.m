controller = NewDrawingController();
controller.setup_controller();
controller.robotController.set_speed_gripper(1000);
controller.robotController.robot_model.kinematic_model.A3 = 11;

controller.grab_pen();

controller.draw_line([10+0.5,17.5+0.5+0.3], [20+0.5+0.4,17.5+0.5], 0, 1);
controller.draw_line([20+0.5+0.4,17.5+0.5], [15+0.3,12.5+0.4], 1, 1);
controller.draw_line([15+0.3,12.5+0.4], [15+0.3,17.5+0.5+0.4], 1, 1);
controller.draw_circle_segment([17.5+0.5,17.5+0.5], 2.5, -pi-0.01, pi/2+0.1, 1, 1, 1);
%controller.draw_line([20.5,17.5+0.5],[20.5,17.5+0.5], 1, 0);
controller.close_controller();
clear all;