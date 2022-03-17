controller = SushiController();
controller.setup_controller();
controller.robotController.robot_model.kinematic_model.A3 = 12;
controller.robotController.set_speed_gripper(35);
% 
% try
% %     
% %     
% catch ME
%     ME
% %     
% end
controller.turn_table(-pi/2);

controller.grab_rice([5,-15]);
controller.place_rice([15,0]);

controller.grab_salmon([]);
controller.place_salmon([]);

controller.turn_table(-0.02);

controller.grab_weed([]);
controller.place_weed([]);

controller.turn_table(-pi/2);

controller.move_sushi(pos_gra, pos_place);

controller.close_controller();

clear all

% 
% controller = SushiController();
% controller.turn_table(-pi/2)

