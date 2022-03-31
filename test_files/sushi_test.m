  controller = SushiController();
controller.setup_controller();
controller.robotController.robot_model.kinematic_model.A3 = 14;
controller.robotController.set_speed_gripper(35);
% 
% try
% %     
% %     
% catch ME
%     ME
% %     
% end
% % 
controller.grab_rice([5,-15]);
controller.place_rice([14,0]);
%   
% % % 
controller.grab_salmon([18.1,18.1,4.5]);
controller.place_salmon([20,0]);
% % % % 
controller.turn_table(-pi/2,[22.5,0]);

controller.grab_weed([0.01,20,4]);
controller.place_weed([20,0]);
% % 
controller.turn_table(0,[15,-7.5]);

controller.move_sushi([14,0], [17.5,-17.5]);

controller.close_controller();

clear all

% 
% controller = SushiController();
% controller.turn_table(-pi/2)

