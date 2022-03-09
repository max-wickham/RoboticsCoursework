%model = KinematicModel();
%model.positions(10,0,10,0)

controller = CubeController();
controller.setup_controller();
% controller.move_servo(5,2000);
% % controller.move_servo(5,2430);
%%controller.set_speed_arm(1000,100);
%controller.set_speed_gripper();
try
        position = [15,15];
        flip_angle = 90;
        controller.flip_on_the_spot(position, flip_angle)
    
   catch ME
        controller.close_controller();
        ME

end 
%trajectory([10,10,10, pi/2],[10,0,10, pi/2])

% polar_to_cartesian([0,0.1,0.2], 5) + [0, 2]
