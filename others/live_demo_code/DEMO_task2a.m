%______2a______%
controller = NewCubeController();
controller.setup_controller();

try
        start1 = [7,-7];
        start2 = [7,2];
        start3 = [0-0.01,7];
        finish4 = [0-0.01,-6];
        finish5 = [4,-4];
        finish6 = [9,0];
        flip_angle = 0;
        finish4_new = [0-0.01,-6.55];
        
        
        controller.robotController.get_current_position()
        %controller.main_cube(start1, finish4, flip_angle);
        %controller.reach_move_flip_higher( start1.*2.5, finish4.*2.5, -pi/2, -pi/2);
        controller.reach_move_flip( start1.*2.5, finish4_new.*2.5, 0.01, -pi/2);
controller.main_cube(start2, finish5, flip_angle);
controller.main_cube(start3, finish6, flip_angle);

        controller.close_controller();

   catch ME
        controller.close_controller();
        ME
end 

clear all

