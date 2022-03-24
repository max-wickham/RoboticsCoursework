controller = NewDrawingController();
controller.setup_controller();
controller.robotController.set_speed_gripper(1000);
controller.robotController.robot_model.kinematic_model.A3 = 11;

controller.grab_pen();

controller.draw_line([10,17.5], [20,17.5], 0, 1);
controller.draw_line([20,17.5], [15,12.5], 1, 1);
controller.draw_line([15,12.5], [15,17.5], 1, 1);
controller.draw_circle_segment([17.5,17.5], 2.5, pi-0.01, -pi/2, 1, 0, 0);

controller.close_controller();
clear all;