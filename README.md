TTT_Baxter
==========

Playing TicTacToe with Baxter

Requirement
-----------
- a usb camera

Usage
-----

1. Run the trajectory controller through baxter_trajectory_controller.launch. It seems that it has now been renamed to Joint Trajectory Action Server.

2. Run tictactoe.launch

    rosrun tictactoe move_maker_server src/tictactoe/resources/trajectories/ttt_trajectories.xml /sdk/robot/limb/left/follow_joint_trajectory
