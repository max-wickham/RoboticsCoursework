classdef CubeController
    properties
        robotController = NewNewRobotController()
        open_servo_val = 2000
        closed_servo_val = 2460
        grip_inwards = -pi+0.04;
        grip_outwards = -0.02;
        grip_vertical = -pi/2;
        UP_level_vert = 15 %8
        DOWN_level_hor = 3.8%FLIPPING 3.5 
        UP_level = 15 %5
        DOWN_level_vert = 4%FLIPPING 4.3 
        grid_to_cm = 2.5
        max_range = 19.8
        min_range = 6
        max_range_stack = 20
        min_range_stack = 17.5
        flip_position = [7.5,-20 ] %0, 17.5
        cube_height = 2.3
    end
    methods

        function main_cube(obj,cube_pos_grid, final_pos_grid, flip_angle)
            %***INPUTS****% 
            % cube pos, final pos = x,y in grid coords
            % flip angle = angle diff in rad (inwards from top) 
            % append height and angle to cube and final pos
        
            %***PROCESS***%
            % convert to cm 
            cube_pos = obj.grid_to_cm*cube_pos_grid;
            final_pos = obj.grid_to_cm*final_pos_grid;
    
            % compute flip given two positions (and robot range), 
            % append to goal positions
            [gripper_initial_angle, gripper_final_angle, flip_on_spot] = obj.set_flip(cube_pos, final_pos, flip_angle);
            
            %--if cannot flip in either position but need to flip
            %move to external pos, flip there, then move to final pos
            if(flip_on_spot == flip_angle && flip_on_spot ~= 0) 
                obj.reach_move_flip(cube_pos, obj.flip_position, obj.grip_vertical, obj.grip_vertical);
                obj.flip_on_the_spot(obj.flip_position, flip_on_spot);
                obj.reach_move_flip(obj.flip_position, final_pos, obj.grip_vertical, obj.grip_vertical);
            else
                % reach cube and move, flip if possible
                obj.reach_move_flip(cube_pos, final_pos, gripper_initial_angle, gripper_final_angle);
                % rest of flipping performed on the spot
                obj.flip_on_the_spot(final_pos, flip_on_spot);
            end 
          
        end
        
        function setup_controller(obj)
            obj.robotController.init();
            obj.robotController.control_mode_setup();
        end

        function close_controller(obj)
            obj.robotController.close();
        end
        
        %--------------------------%
        %return initial and final gripper orientation plus required flip on spot       
        function [gripper_initial_angle, gripper_final_angle, flip_on_spot] = set_flip(obj, cube_pos, final_pos, flip_angle)
            
            %**INPUTS**
            %pos is x,y coords
            %flip_angle is amount of flip requested in outwards direction from top
            % 0 deg, no flip
            % allowed values for flip = 0, 90,180,270
            
            %**OUTPUTS**
            %grip initial is angle at which grab cube, 
            %grip final is angle at which release the cube
            %flip_on_spot if couldnt perform all turn, flip still to perform a final position
            
            dir_init = obj.turnability(cube_pos);
            dir_final = obj.turnability(final_pos);
            flip_on_spot = 0;
            
            gripper_initial_angle =0; 
            gripper_final_angle = 0; 
            %********compute flipping mode
            %--90deg turn
            if flip_angle == 90 %neg direction
                if dir_init(2) % neg allowed
                    gripper_initial_angle = obj.grip_inwards;
                    gripper_final_angle = obj.grip_vertical;
                elseif dir_final(1)
                    gripper_final_angle = obj.grip_outwards;
                    gripper_initial_angle = obj.grip_vertical;
                else
                    gripper_initial_angle = obj.grip_vertical;
                    gripper_final_angle = obj.grip_vertical;
                    flip_on_spot = 90;
                end
            %--180deg turn
            elseif flip_angle == 180
                if dir_init(2) && dir_final(1) % neg allowed
                    gripper_initial_angle = obj.grip_inwards;
                    gripper_final_angle = obj.grip_outwards;
                elseif dir_init(1) && dir_final(2)
                    gripper_final_angle = obj.grip_inwards;
                    gripper_initial_angle = obj.grip_outwards;
                else
                    % TODO
                    % recursion or back 90/270
                    gripper_init1, gripper_final1, flip_on_spot1 = set_flip(cube_pos, final_pos, 90);
                    gripper_init2, gripper_final2, flip_on_spot2 = set_flip(cube_pos, final_pos, 270);
                    if flip_on_spot1 == 0 %can do at least 90
                        gripper_initial_angle = gripper_init1;
                        gripper_final_angle = gripper_final1;
                        flip_on_spot = 90;
                    elseif flip_on_spot2 == 0 % can do at least 270
                        gripper_initial_angle = gripper_init2;
                        gripper_final_angle = gripper_final2;
                        flip_on_spot = 270;  
                    else
                        gripper_initial_angle = obj.grip_vertical;
                        gripper_final_angle = obj.grip_vertical;
                        flip_on_spot = 180;
                    end
                end
            %--270deg turn
            elseif flip_angle == 270 %pos direction
                if dir_init(1) % pos allowed
                    gripper_initial_angle = obj.grip_outwards;
                    gripper_final_angle = obj.grip_vertical;
                elseif dir_final(2)
                    gripper_final_angle = obj.grip_inwards;
                    gripper_initial_angle = obj.grip_vertical;
                else
                    gripper_initial_angle = obj.grip_vertical;
                    gripper_final_angle = obj.grip_vertical;
                    flip_on_spot = 270;
                end
            elseif flip_angle == 0
                gripper_initial_angle = obj.grip_vertical;
                    gripper_final_angle = obj.grip_vertical;
                    flip_on_spot = 0;
            end
            
        end

        
        function reach_move_flip(obj,cube_pos, final_pos, gripper_initial_angle, gripper_final_angle)

            %****REACH: open, up, new position and gripper orientation, down, close
            obj.open_gripper();
            %---current_pos = obj.robotController.get_current_position();
            current_pos = obj.robotController.get_current_position()%[5,0, obj.UP_level_vert, obj.grip_vertical];
            %up
            obj.robotController.move_to_positions([[current_pos(1), current_pos(2), obj.UP_level_vert, current_pos(4)]]);           
            %reach
            up_grip_pos = [cube_pos(1), cube_pos(2), obj.UP_level_vert,obj.grip_vertical ];
%             pos_array = obj.robotController.trajectory(current_pos, up_grip_pos);
%             obj.robotController.move_to_positions(pos_array);
            obj.robotController.scale = 0.6;
            obj.robotController.move_to_positions([up_grip_pos]);
            orient_grip_pos = [cube_pos(1), cube_pos(2), obj.UP_level_vert, gripper_initial_angle];
            pos_array = obj.robotController.trajectory_angle(up_grip_pos, orient_grip_pos); %angle obj.robotController.traj
            obj.robotController.move_to_positions(pos_array);
            %down
            if gripper_initial_angle == obj.grip_vertical
                DOWN = obj.DOWN_level_vert;
            else
                DOWN = obj.DOWN_level_hor;
            end
            obj.robotController.move_to_positions([[cube_pos(1), cube_pos(2), DOWN, gripper_initial_angle]]);
            obj.close_gripper();
            obj.robotController.get_current_position()
            
            %****MOVE CUBE: up, new position and orientation, down, open, up
            %up
            %--current_pos = obj.robotController.get_current_position();
            current_pos = [cube_pos(1), cube_pos(2), DOWN, gripper_initial_angle];
            pos = trajectory(current_pos, [current_pos(1), current_pos(2),  current_pos(3)+1 , current_pos(4)]);
            obj.robotController.move_to_positions(pos);
            obj.robotController.move_to_positions([current_pos(1), current_pos(2),  obj.UP_level , current_pos(4)]);
            %move
            vertical_cube_pos = [current_pos(1), current_pos(2), obj.UP_level, obj.grip_vertical];
            pos_array = obj.robotController.trajectory_angle([current_pos(1), current_pos(2), obj.UP_level, current_pos(4)], vertical_cube_pos);
            obj.robotController.move_to_positions(pos_array);
            up_cube_pos = [final_pos(1), final_pos(2), obj.UP_level, obj.grip_vertical];
%             pos_array = obj.robotController.trajectory(vertical_cube_pos, up_cube_pos);
%             obj.robotController.move_to_positions(pos_array);
            obj.robotController.scale = 0.6;
            obj.robotController.move_to_positions([up_cube_pos]);
            orient_cube_pos = [final_pos(1), final_pos(2), obj.UP_level, gripper_final_angle];
            pos_array = obj.robotController.trajectory_angle(up_cube_pos, orient_cube_pos);
            obj.robotController.move_to_positions(pos_array);

            %down
            if gripper_final_angle == obj.grip_vertical
                DOWN = obj.DOWN_level_vert;
            else
                DOWN = obj.DOWN_level_hor;
            end
            obj.robotController.move_to_positions([[final_pos(1), final_pos(2), DOWN, gripper_final_angle]]);
            obj.open_gripper();
            %up
            obj.robotController.move_to_positions([[final_pos(1), final_pos(2), obj.UP_level, gripper_final_angle]]);
                
        end  


        function flip_on_the_spot(obj,position, flip_angle)

            %**INPUT**
            %pos is x,y coords
            %flip_angle is amount of flip requested in outwards direction from top
            % 0 deg is red face up, no flip
            % allowed values for flip = 0, 90,180,270
            dir = obj.turnability(position);
            %********compute flipping mode
            %turn = flip_angle/90;
            turn = 0;

            if flip_angle == 0
                return
            elseif flip_angle == 90 %neg direction
                if dir(2) % neg allowed
                    gripper_final_angle = obj.grip_inwards;
                    turn = 1;
                elseif dir(1)
                    gripper_final_angle = obj.grip_outwards;
                    turn = 3;
                end
            elseif flip_angle == 180 %180°
                if dir(2) && dir(1) % neg allowed
                    gripper_final_angle = obj.grip_inwards;
                    turn = 1;
                elseif dir(2)
                    gripper_final_angle = obj.grip_inwards;
                    turn = 2;
                elseif dir(1)
                    gripper_final_angle = obj.grip_outwards;
                    turn = 2;   
                end
            elseif flip_angle == 270 %pos direction
                if dir(1) % neg allowed
                    gripper_final_angle = obj.grip_outwards;
                    turn = 1;
                elseif dir(2)
                    gripper_final_angle = obj.grip_inwards;
                    turn = 3;
                end         
            end
            %********actual flipping
            %open and go up, turn gripper, down, close, go up, turn, down, open,
            %up
            
            if dir(1) == 0 & dir(2) == 0 & flip_angle ~= 0
                message = 'cannot flip, move to new position anf flip'
                obj.reach_move_flip( position, obj.flip_position, obj.grip_vertical,obj.grip_vertical);
                obj.flip_on_the_spot( obj.flip_position, flip_angle);
                obj.reach_move_flip( obj.flip_position,position, obj.grip_vertical,obj.grip_vertical);
            else
              %adjustment of scentrato 
                for i=1:turn
                    %orient
                    pos = obj.robotController.trajectory_angle([position(1), position(2), obj.UP_level, obj.grip_vertical],[position(1), position(2), obj.UP_level, gripper_final_angle]);
                    obj.robotController.move_to_positions(pos);
                    %down and close
                    if gripper_final_angle == obj.grip_vertical
                        DOWN = obj.DOWN_level_vert;
                    else
                        DOWN = obj.DOWN_level_hor;
                    end
                    obj.robotController.move_to_positions([[position(1), position(2), DOWN, gripper_final_angle]]);
                    obj.close_gripper();
                    %
                    obj.robotController.move_to_positions([[position(1), position(2), obj.UP_level, gripper_final_angle]]);
                    if flip_angle == 180 && turn == 1
                        pos = obj.robotController.trajectory([position(1), position(2), obj.UP_level, gripper_final_angle], [position(1), position(2), obj.UP_level, obj.grip_outwards]);
                        obj.robotController.move_to_positions(pos);
                        obj.robotController.move_to_positions([[position(1), position(2), obj.DOWN_level_hor, obj.grip_outwards]]);
                        obj.open_gripper();
                        obj.robotController.move_to_positions([[position(1), position(2), obj.UP_level, obj.grip_outwards]]);
                    else
%                           if (gripper_final_angle == obj.grip_vertical)
                              vec = position / norm(position) * 0.6; 
                              angle_off = 0;
%                           else 
%                               vec = [0,0];
                          %end
                        %--pos_current = obj.robotController.get_current_position()
                        if norm(position) > 21.3
                            angle_off = 0.45;
                        else 
                            angle_off = 0;
                        end 
                        pos_current = [position(1)+vec(1), position(2)+vec(2), obj.UP_level, gripper_final_angle]
                        pos = obj.robotController.trajectory_angle(pos_current,[pos_current(1)+vec(1), pos_current(2)+vec(2), pos_current(3),  obj.grip_vertical+ angle_off]);
                        obj.robotController.move_to_positions(pos);
                        
                        obj.robotController.move_to_positions([[position(1)+vec(1), position(2)+vec(2), obj.DOWN_level_vert+0.3, obj.grip_vertical+ angle_off]]);
                        obj.open_gripper();
                        obj.robotController.move_to_positions([[position(1)+vec(1), position(2)+vec(2), obj.UP_level,obj.grip_vertical+angle_off ]]);
                    end
                end 
            end 
        end 
        
        
        %given position it returns turning dir=(pos dir allowed, neg dir allowed)
        function dir = turnability(obj, pos)
        %pos is x,y coords
            if norm(pos) < obj.min_range
                dir = [false, true]; %neg allowed
            elseif norm(pos) > obj.max_range
                dir = [true, false]; %pos allowed
            else 
                dir = [false, false]; %in range
            end
        end
        
        function [vert_stack, horiz_stack] = turnability_stack(obj, pos)
            %could add different condition for 2 or 3 cubes stacks - stack_num
            if norm(pos) < obj.max_range_stack
                        vert_stack = true; %nvertical grip allowed
            else
                        vert_stack = false;
            end 
            if norm(pos) > obj.min_range_stack
                        horiz_stack = true; %outwards horizontal grip allowed
            else 
                        horiz_stack = false;
            end
        end

        function move_up(obj, pos)
            %pos array of 4 coords
            pos(3)= obj.UP_level;
            current_pos = obj.robotController.get_current_position()
            pos_array = trajectory(current_pos, pos);
            % obj.robotController.move_to_positions([pos]);
            obj.robotController.move_to_positions(pos_array);
        end

        function move_down(obj, pos)
            %pos array of 4 coords
            if pos(4) == obj.grip_vertical
                DOWN = obj.DOWN_level_vert;
            else
                DOWN = obj.DOWN_level_hor;
            end
            pos(3)= DOWN;
            % obj.robotController.move_to_positions([pos]);
            current_pos = obj.robotController.get_current_position()
            pos_array = trajectory(current_pos, pos);
            % obj.robotController.move_to_positions([pos]);
            obj.robotController.move_to_positions(pos_array);
        end

        function open_gripper(obj)
            obj.robotController.set_speed_gripper(500);
            obj.robotController.move_servo(5,obj.open_servo_val);
        end

        function close_gripper(obj)
            obj.robotController.set_speed_gripper(35);
            obj.robotController.move_servo(5,obj.closed_servo_val);
        end
    
        function main_cube_stack(obj,start_pos, final_pos_grid, flip_angle, stack_num)
            %INPUTS
            %stack_num is cube number moved in finished pos
            if stack_num == 1
                obj.main_cube(start_pos, finish_pos, flip_angle);
            else
                %***INPUTS****% 
                % cube pos, final pos = x,y in grid coords
                % flip angle = angle diff in rad (inwards from top) 
                % append height and angle to cube and final pos

                %***PROCESS***%
                % convert to cm 
                cube_pos = obj.grid_to_cm*start_pos;
                final_pos = obj.grid_to_cm*final_pos_grid;

                %--check if angles are compatible with stacking
                dir_init = obj.turnability(cube_pos);
                [vert_stack, horiz_stack] = obj.turnability_stack(final_pos);
                 
                if(vert_stack)
                        gripper_final_angle = obj.grip_vertical;
                        gripper_initial_angle = obj.grip_vertical;
                elseif(horiz_stack && dir_init(1))
                        gripper_final_angle = obj.grip_outwards;
                        gripper_initial_angle = obj.grip_outwards;   
                elseif(~dir_init(1))
                            message = 'need to use third position and flip cube first'
                            obj.reach_move_flip(cube_pos, obj.flip_position, obj.grip_vertical,obj.grip_vertical);
                            obj.reach_move_flip_stack(obj.flip_position,final_pos,  obj.grip_outwards,obj.grip_outwards, stack_num);   
                end   
                
                % gripper angle 
                if flip_angle == 0 
                        obj.reach_move_flip_stack(cube_pos, final_pos, gripper_initial_angle, gripper_final_angle, stack_num);
                else
                    if(dir_init(1) | dir_init(2))
                        obj.flip_on_the_spot(cube_pos, flip_angle);
                        obj.reach_move_flip_stack(cube_pos, final_pos, gripper_initial_angle, gripper_final_angle, stack_num);
                    else 
                        obj.reach_move_flip(cube_pos, obj.flip_position, gripper_initial_angle, gripper_initial_angle);
                        obj.flip_on_the_spot(obj.flip_position, flip_angle);
                        obj.reach_move_flip_stack(obj.flip_position, final_pos, gripper_initial_angle, gripper_final_angle, stack_num);
                    end 
                end
            end   
        end
        
        function reach_move_flip_stack(obj,cube_pos, final_pos, gripper_initial_angle, gripper_final_angle, stack_num)
            %****REACH: open, up, new position and gripper orientation, down, close
            obj.open_gripper();
            %---current_pos = obj.robotController.get_current_position();
            current_pos = obj.robotController.get_current_position()%[5,0, obj.UP_level_vert, obj.grip_vertical];
            %up
            obj.robotController.move_to_positions([[current_pos(1), current_pos(2), obj.UP_level_vert, current_pos(4)]]);           
            %reach
            up_grip_pos = [cube_pos(1), cube_pos(2), obj.UP_level_vert,obj.grip_vertical ];
%             pos_array = obj.robotController.trajectory(current_pos, up_grip_pos);
%             obj.robotController.move_to_positions(pos_array);
            obj.robotController.scale = 0.6;
            obj.robotController.move_to_positions([up_grip_pos]);
            orient_grip_pos = [cube_pos(1), cube_pos(2), obj.UP_level_vert, gripper_initial_angle];
            pos_array = obj.robotController.trajectory_angle(up_grip_pos, orient_grip_pos); %angle obj.robotController.traj
            obj.robotController.move_to_positions(pos_array);
            %down
            if gripper_initial_angle == obj.grip_vertical
                DOWN = obj.DOWN_level_vert;
            else
                DOWN = obj.DOWN_level_hor;
            end
            obj.robotController.move_to_positions([[cube_pos(1), cube_pos(2), DOWN, gripper_initial_angle]]);
            obj.close_gripper();
            
            %****MOVE CUBE: up, new position, down, open, up
            %up
            %--current_pos = obj.robotController.get_current_position();
            current_pos = [cube_pos(1), cube_pos(2), DOWN, gripper_initial_angle];
            pos = trajectory(current_pos, [current_pos(1), current_pos(2),  current_pos(3)+1 , current_pos(4)]);
            obj.robotController.move_to_positions(pos);
            obj.robotController.move_to_positions([current_pos(1), current_pos(2),  obj.UP_level , current_pos(4)]);
            %move
            vertical_cube_pos = [current_pos(1), current_pos(2), (obj.UP_level), gripper_final_angle];
            pos_array = obj.robotController.trajectory_angle([current_pos(1), current_pos(2), (obj.UP_level), current_pos(4)], vertical_cube_pos);
            obj.robotController.move_to_positions(pos_array);
            vec = final_pos / norm(final_pos) * (stack_num-1)*0.1;
            final_pos = final_pos - vec;
            up_cube_pos = [final_pos(1), final_pos(2), (obj.UP_level), gripper_final_angle];
%             pos_array = obj.robotController.trajectory(vertical_cube_pos, up_cube_pos);
%             obj.robotController.move_to_positions(pos_array);
            obj.robotController.scale = 0.6;
            obj.robotController.move_to_positions([up_cube_pos]);
%             orient_cube_pos = [final_pos(1), final_pos(2), (obj.UP_level), gripper_final_angle];
%             pos_array = obj.robotController.trajectory(up_cube_pos, orient_cube_pos);
%             obj.robotController.move_to_positions(pos_array);

            %down
            if stack_num == 2
                adj = 0.0;
            else
                adj = 0;
            end 
            if gripper_final_angle == obj.grip_vertical
                DOWN = obj.DOWN_level_vert+(stack_num-1-adj)*obj.cube_height;
            else
                DOWN = obj.DOWN_level_hor+(stack_num-1-adj)*obj.cube_height;
            end
            obj.robotController.move_to_positions([[final_pos(1), final_pos(2), DOWN, gripper_final_angle]]);
            obj.open_gripper();
            %up
            obj.robotController.move_to_positions([[final_pos(1), final_pos(2),(obj.UP_level) , gripper_final_angle]]);
              
        end

        
    end
end