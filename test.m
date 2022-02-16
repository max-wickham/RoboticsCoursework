%model = KinematicModel();
%model.positions(10,0,10,0)

controller = RobotController();
controller.init();
controller.control_mode_setup();
pos = [20,0,20, 0; 20,20,20,0 ];
controller.move_to_positions(pos);
%controller.move_servo(2, 3072);
%model2 = KinematicModel();
%model2.angles(20,0,20,0)
%model = RobotModel();
%model.servo_vals([20,0,20],0)
controller.close();