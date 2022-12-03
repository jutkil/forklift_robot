%clear,close all;
rosshutdown
rosinit('192.168.40.130',11311)
%rostopic list

pub_joints = rospublisher('/forklift_arm_controller/command','trajectory_msgs/JointTrajectory');

t = 0:0.05:2;



q0 = deg2rad([0 0 0 0 0 0]);
q_before_picking_pallet = deg2rad([-90 153.5 -22.5 -90 84 -90]);
q_push_into_pallet = deg2rad([-90 171 30 -90 54 -90]);
q_tilt_pallet_up = deg2rad([-90 170 30 -90 57 -90]);
q_tilt_pallet_down = deg2rad([-90 171 30 -90 54 -90])
q_pullout_from_pallet = deg2rad([-90 155 -22.5 -90 84 -90]);
q_after_picking_pallet = deg2rad([-90 153.5 -22.5 -90 90 -90]);
q_lift_pallet_up = deg2rad([-90 90 -22.5 -22.5 91 -90]);
q_drivingpos = deg2rad([90 105 -45 -50 100 -90]);

q_reolpos_1 = deg2rad([0 0 0 0 0 -90]);
q_reolpos_2 = deg2rad([0 45 30 0 19 -90]);
q_push_in_reol = deg2rad([0 90 20 -56 19 -90]);
q_tilt_down_reol = deg2rad([0 90 20 -52 22 -90]);
q_reolpos_3 = deg2rad([0 45 30 0 19 -90]);

q_showoff_pos = deg2rad([-90 0 0 0 0 -90]);
q_showoff_pos_hoyre = deg2rad([-90 0 0 0 0 0]);
q_showoff_pos_venstre = deg2rad([-90 0 0 0 0 -180]);

joint_send = rosmessage('trajectory_msgs/JointTrajectoryPoint')
%joint_send.Positions = deg2rad([90;105;-45;-50;100;-90]);

% 
% java.lang.Thread.sleep(7000);
 joint_send.Positions = q_drivingpos;
 joint_send.TimeFromStart.Sec = 5
 msg = rosmessage(pub_joints);
 msg.JointNames={'l1' 'l2' 'l3' 'l4' 'l5' 'l6'};
 msg.Points = joint_send
 send(pub_joints,msg)

 %% plukke palle fra bakken
 
 
java.lang.Thread.sleep(7000);

joint_send.Positions = q_lift_pallet_up;
joint_send.TimeFromStart.Sec = 5
msg = rosmessage(pub_joints);
msg.JointNames={'l1' 'l2' 'l3' 'l4' 'l5' 'l6'};
msg.Points = joint_send
send(pub_joints,msg)

java.lang.Thread.sleep(7000);

joint_send.Positions = q_before_picking_pallet;
joint_send.TimeFromStart.Sec = 5
msg = rosmessage(pub_joints);
msg.JointNames={'l1' 'l2' 'l3' 'l4' 'l5' 'l6'};
msg.Points = joint_send
send(pub_joints,msg)

java.lang.Thread.sleep(7000);

joint_send.Positions = q_push_into_pallet;
joint_send.TimeFromStart.Sec = 5
msg = rosmessage(pub_joints);
msg.JointNames={'l1' 'l2' 'l3' 'l4' 'l5' 'l6'};
msg.Points = joint_send
send(pub_joints,msg)

java.lang.Thread.sleep(7000);

joint_send.Positions = q_tilt_pallet_up;
joint_send.TimeFromStart.Sec = 1
msg = rosmessage(pub_joints);
msg.JointNames={'l1' 'l2' 'l3' 'l4' 'l5' 'l6'};
msg.Points = joint_send
send(pub_joints,msg)

java.lang.Thread.sleep(2000);


joint_send.Positions = q_after_picking_pallet;
joint_send.TimeFromStart.Sec = 5
msg = rosmessage(pub_joints);
msg.JointNames={'l1' 'l2' 'l3' 'l4' 'l5' 'l6'};
msg.Points = joint_send
send(pub_joints,msg)

java.lang.Thread.sleep(7000);


joint_send.Positions = q_lift_pallet_up;
joint_send.TimeFromStart.Sec = 5
msg = rosmessage(pub_joints);
msg.JointNames={'l1' 'l2' 'l3' 'l4' 'l5' 'l6'};
msg.Points = joint_send
send(pub_joints,msg)

java.lang.Thread.sleep(7000);

joint_send.Positions = q_drivingpos;
joint_send.TimeFromStart.Sec = 5
msg = rosmessage(pub_joints);
msg.JointNames={'l1' 'l2' 'l3' 'l4' 'l5' 'l6'};
msg.Points = joint_send
send(pub_joints,msg)

java.lang.Thread.sleep(7000);



Reolplassering
joint_send.Positions = q_drivingpos;
joint_send.TimeFromStart.Sec = 5
msg = rosmessage(pub_joints);
msg.JointNames={'l1' 'l2' 'l3' 'l4' 'l5' 'l6'};
msg.Points = joint_send
send(pub_joints,msg)
 java.lang.Thread.sleep(7000);

joint_send.Positions = q_reolpos_1;
joint_send.TimeFromStart.Sec = 5
msg = rosmessage(pub_joints);
msg.JointNames={'l1' 'l2' 'l3' 'l4' 'l5' 'l6'};
msg.Points = joint_send
send(pub_joints,msg)
java.lang.Thread.sleep(7000);

joint_send.Positions = q_reolpos_2;
joint_send.TimeFromStart.Sec = 6
msg = rosmessage(pub_joints);
msg.JointNames={'l1' 'l2' 'l3' 'l4' 'l5' 'l6'};
msg.Points = joint_send
send(pub_joints,msg)
java.lang.Thread.sleep(7000);

joint_send.Positions = q_push_in_reol;
joint_send.TimeFromStart.Sec = 6
msg = rosmessage(pub_joints);
msg.JointNames={'l1' 'l2' 'l3' 'l4' 'l5' 'l6'};
msg.Points = joint_send
send(pub_joints,msg)
java.lang.Thread.sleep(6000);

joint_send.Positions = q_tilt_down_reol;
joint_send.TimeFromStart.Sec = 2
msg = rosmessage(pub_joints);
msg.JointNames={'l1' 'l2' 'l3' 'l4' 'l5' 'l6'};
msg.Points = joint_send
send(pub_joints,msg)
java.lang.Thread.sleep(4000);

joint_send.Positions = q_reolpos_3;
joint_send.TimeFromStart.Sec = 5
msg = rosmessage(pub_joints);
msg.JointNames={'l1' 'l2' 'l3' 'l4' 'l5' 'l6'};
msg.Points = joint_send
send(pub_joints,msg)
java.lang.Thread.sleep(7000);

joint_send.Positions = q_drivingpos;
joint_send.TimeFromStart.Sec = 5
msg = rosmessage(pub_joints);
msg.JointNames={'l1' 'l2' 'l3' 'l4' 'l5' 'l6'};
msg.Points = joint_send
send(pub_joints,msg)
java.lang.Thread.sleep(7000);




show off
java.lang.Thread.sleep(5000);

joint_send.Positions = q_drivingpos;
joint_send.TimeFromStart.Sec = 5
msg = rosmessage(pub_joints);
msg.JointNames={'l1' 'l2' 'l3' 'l4' 'l5' 'l6'};
msg.Points = joint_send
send(pub_joints,msg)
java.lang.Thread.sleep(7000);

joint_send.Positions = q_showoff_pos;
joint_send.TimeFromStart.Sec = 5
msg = rosmessage(pub_joints);
msg.JointNames={'l1' 'l2' 'l3' 'l4' 'l5' 'l6'};
msg.Points = joint_send
send(pub_joints,msg)

java.lang.Thread.sleep(8000);

joint_send.Positions = q_showoff_pos_hoyre;
joint_send.TimeFromStart.Sec = 3
msg = rosmessage(pub_joints);
msg.JointNames={'l1' 'l2' 'l3' 'l4' 'l5' 'l6'};
msg.Points = joint_send
send(pub_joints,msg)

java.lang.Thread.sleep(4000);

joint_send.Positions = q_showoff_pos_venstre;
joint_send.TimeFromStart.Sec = 3
msg = rosmessage(pub_joints);
msg.JointNames={'l1' 'l2' 'l3' 'l4' 'l5' 'l6'};
msg.Points = joint_send
send(pub_joints,msg)

java.lang.Thread.sleep(4000);

joint_send.Positions = q_showoff_pos;
joint_send.TimeFromStart.Sec = 3
msg = rosmessage(pub_joints);
msg.JointNames={'l1' 'l2' 'l3' 'l4' 'l5' 'l6'};
msg.Points = joint_send
send(pub_joints,msg)

java.lang.Thread.sleep(4000);

joint_send.Positions = q_drivingpos;
joint_send.TimeFromStart.Sec = 5
msg = rosmessage(pub_joints);
msg.JointNames={'l1' 'l2' 'l3' 'l4' 'l5' 'l6'};
msg.Points = joint_send
send(pub_joints,msg)
java.lang.Thread.sleep(7000);