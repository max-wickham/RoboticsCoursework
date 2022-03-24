%--2.b pick and flip and move--%

controller = NewCubeController();
controller.setup_controller();
% controller.robotController.set_speed_gripper(100);

try
        start1 = [7,-7];
        start2 = [7,2];
        start3 = [0-0.01,7];
        finish4 = [0-0.01,-6];
        finish5 = [4,-4];
        finish6 = [9,0];
        flip_angle = 0;
        
        pos = controller.robotController.get_current_position()
        %move_up(pos)
        controller.open_gripper();

        controller.flip_7_7_270();
        controller.flip_72_270();  
        controller.flip_72_270(); 
         controller.flip_07_90();
        %? need to move in different pos ?
%          controller.flip_on_the_spot(start3, 270 );

        controller.close_controller();

   catch ME
        controller.close_controller();
        ME
end 
clear all
