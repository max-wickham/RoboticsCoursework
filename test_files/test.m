%model = KinematicModel();
%model.positions(10,0,10,0)

controller = NewRobotController();
controller.init();
controller.control_mode_setup();
% controller.move_servo(5,2000);
% % controller.move_servo(5,2430);
%%controller.set_speed_arm(1000,100);
%controller.set_speed_gripper();
try
    %pos = trajectory_angle([20,0,, -pi/2],[ 20,0,5,-0.05]);
    current_pos = controller.get_current_position()
    pos1 = trajectory(current_pos,[17.5,0,3, -pi/2]);
     pos2 = trajectory([17.5,0,3, -pi/2],[5 ,0,3,-pi/2]);
     pos = [pos1;pos2]
% pos = pos1
%     pos = trajectory_angle([17.5,0,3, -pi/2],[17.5 ,0,3, -pi/2]);
   controller.move_to_positions(pos);
%     pos = trajectory([17.5,0,3, -pi/2],[5 ,0,3,-pi/2]);
    
    %----problem with this move, going the otehr way: controller.move_to_positions([10,10,10, -pi/2; 16,16,0,0]);
%      controller.move_to_positions(pos);
    % controller.move_servo(5,2000);
    % controller.move_servo(5,2000);
    t = controller.get_current_position()
    %controller.readPID();
%    controller.test_move_to_positions([0,pi/2,-pi/2,pi/2]);
    controller.close();
    catch ME
        controller.close();
        ME

end
clear all
%trajectory([10,10,10, pi/2],[10,0,10, pi/2])

% polar_to_cartesian([0,0.1,0.2], 5) + [0, 2]

    %controller.move_servo(2, 3072);
    %model2 = KinematicModel();
    %model2.angles(20,0,20,0)
    %model = RobotModel();
    %model.servo_vals([20,0,20],0)
        %controller.move_to_positions(pos);
    %%controller.set_speed_arm(3000, 100);
    %%controller.move_to_positions(pos);
    % controller.move_servo(5,2430);
