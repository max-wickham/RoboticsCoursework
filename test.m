%model = KinematicModel();
%model.positions(10,0,10,0)

controller = RobotController();
controller.init();
controller.control_mode_setup();
pos = [10,0,10, 0; 10,0,10,0 ];
%controller.move_to_positions(pos);
controller.move_servo(2, 2048)
controller.close()
