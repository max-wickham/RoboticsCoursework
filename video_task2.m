%----video task 2 cubes----%

%--2.a pick and replace--%
% controller = CubeController();
% controller.setup_controller();
% 
% try
%         start1 = [3,-8];
%         start2 = [9,0];
%         start3 = [6,6];
%         finish4 = [5,-5];
%         finish5 = [4,0];
%         finish6 = [0-0.1,5];
%         flip_angle = 0;
%         
%         
%         controller.robotController.get_current_position()
%         %can also just use reach and move
%         controller.main_cube(start1, finish4, flip_angle);
%         controller.main_cube(start2, finish5, flip_angle);
%         controller.main_cube(start3, finish6, flip_angle);
% 
%         controller.close_controller();
% 
%    catch ME
%         controller.close_controller();
%         ME
% end 
% clear all

%--2.b pick and flip and move--%

% controller = CubeController();
% controller.setup_controller();

% try
%         start1 = [3*2.5,-8*2.5];
%         start2 = [9*2.5,0];
%         start3 = [6*2.5,6*2.5];
%         
%         pos = controller.robotController.get_current_position()
%         %move_up(pos)
%         controller.open_gripper();
%         controller.flip_on_the_spot(start1, 90);
%         
%         controller.flip_on_the_spot(start2, 180);
%         %? need to move in different pos ?
%         controller.flip_on_the_spot(start3, 90);
% 
%         controller.close_controller();
% 
%    catch ME
%         controller.close_controller();
%         ME
% end 
% clear all


%--2.c pick flip and stack--%

controller = CubeController();
controller.setup_controller();

try
        start1 = [3,-8];
        start2 = [9,0];
        start3 = [6,6];
        finish4 = [5,-5];
        finish5 = [4,0];
        finish6 = [0-0.1,5];
        flip_angle = 0;
        
        
        controller.robotController.get_current_position()

        controller.main_cube(start1, finish4, 0);
        %try if horizontal in loc 4 works else fliponspot then stack
        controller.main_cube_stack(start2, finish4, 90, 2);
        controller.main_cube_stack(start3, finish4, 0, 3);

        controller.close_controller();

   catch ME
        controller.close_controller();
        ME
end 
clear all