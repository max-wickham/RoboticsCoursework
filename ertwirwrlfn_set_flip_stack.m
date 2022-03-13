

% to delete later


function [gripper_initial_angle, gripper_final_angle, flip_on_spot][] = set_flip_stack(obj, cube_pos, final_pos, flip_angle, stack_num)
            
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
            [vert_stack, horiz_stack] = obj.turnability_stack(final_pos, stack_num);
            flip_on_spot = 0;
            
            gripper_initial_angle =obj.grip_outwards; 
            gripper_final_angle = obj.grip_outwards; 
            
            if()
            %********compute flipping mode
            %--90deg turn
            if flip_angle == 90 %neg direction
                if dir_init(2) && vert_stack % neg allowed
                    gripper_initial_angle = obj.grip_inwards;
                    gripper_final_angle = obj.grip_vertical;
                elseif dir_final(1) && vert_stack
                    gripper_final_angle = obj.grip_outwards;
                    gripper_initial_angle = obj.grip_vertical;
                else
                    gripper_initial_angle = obj.grip_outwards;
                    gripper_final_angle = obj.grip_outwards;
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
