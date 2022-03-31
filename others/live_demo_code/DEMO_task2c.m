%--2.c pick flip and stack--%

controller = NewCubeController();
controller.setup_controller();

% try
        start1 = [7,-7];
        start2 = [7,2];
        start3 = [0-0.01,7];
        finish4 = [0-0.01,-6];
                finish4_new = [0-0.01,-6.53];
        finish5 = [4,-4];
        finish6 = [9,0];
        flip_angle = 0;
        
%         controller.close_gripper();
%         controller.open_gripper();
         
%         controller.robotController.get_current_position()
         controller.flip_7_7_270();
%         controller.flip_7_7_270();
        %can also just use reach and move
        %controller.main_cube(start1, finish4, flip_angle);
        
%         %try if horizontal in loc 4 works else fliponspot then stack 
%         controller.flip_72_90(); 
 controller.reach_move_flip( start1.*2.5, finish4_new.*2.5, 0.01, -pi/2);
  controller.flip_72_90();
        controller.main_cube_stack(start2, finish4, 0,2);
         controller.main_cube_stack(start3, finish4, 0, 3);
                %controller.main_cube_stack(start1, finish4, 0, 3);
%         

        controller.close_controller();
% 
%    catch ME
%         controller.close_controller();
%         ME
% end 
clear all