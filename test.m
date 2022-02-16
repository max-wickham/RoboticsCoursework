%model = KinematicModel();
%model.positions(10,0,10,0)

controller = RobotController();
controller.init();
controller.control_mode_setup();


controller.close()
