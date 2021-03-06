%----video task 2 cubes----%

%--2.a pick and replace--%
controller = NewCubeController();
controller.setup_controller();

try
        start1 = [3,-8];
        start2 = [9,0];
        start3 = [6,6];
        finish4 = [5,-5];
        finish5 = [4,0];
        finish6 = [0-0.1,4];
        flip_angle = 0;
        
        
        controller.robotController.get_current_position()
        %can also just use reach and move
        controller.main_cube(start1, finish4, flip_angle);
        controller.main_cube(start2, finish5, flip_angle);
        controller.main_cube(start3, finish6, flip_angle);

        controller.close_controller();

   catch ME
        controller.close_controller();
        ME
end 

clear all

%--2.b pick and flip and move--%
% 
% controller = NewCubeController();
% controller.setup_controller();
% % controller.robotController.set_speed_gripper(100);
% 
% try
%         start1 = [3*2.5,-8*2.5];
%         start2 = [9*2.5,0];
% %         start2 = [9*2.5,0];
%         start3 = [6*2.5,-6*2.5];
%         
%         pos = controller.robotController.get_current_position()
%         %move_up(pos)
%         controller.open_gripper();
% %         controller.flip_on_the_spot(start1, 90);
% %         controller.flip_on_the_spot(start2, 180);
%         controller.flip_3_8_90();
%         controller.flip_90_270();  
%         controller.flip_90_270();
%         controller.flip_66_90();
%         %? need to move in different pos ?
% %          controller.flip_on_the_spot(start3, 270 );
% 
%         controller.close_controller();
% 
%    catch ME
%         controller.close_controller();
%         ME
% end 
% clear all

% 
% %--2.c pick flip and stack--%
% 
% controller = NewCubeController();
% controller.setup_controller();
% 
% % try
%         start1 = [3,-8];
%         start2 = [9,0];
%         start3 = [6,6];
%         finish4 = [5,-5];
%         finish4_adj = [5-0.05/2.5, -5+0.05/2.5]
%         finish5 = [4,0];
%         finish6 = [0-0.1,5];
%         flip_angle = 0;
%         
% %         controller.close_gripper();
% %         controller.open_gripper();
%          
%         controller.robotController.get_current_position()
%         %can also just use reach and move
%         controller.main_cube(start1, finish4, flip_angle);
%         %try if horizontal in loc 4 works else fliponspot then stack 
%         controller.flip_90_90();
%         controller.main_cube_stack(start2, finish4, 0, 2);
%        controller.main_cube_stack(start3, finish4_adj, 0, 3);
% 
%         controller.close_controller();
% % 
% %    catch ME
% %         controller.close_controller();
% %         ME
% % end 
% clear all