# TTT_Baxter

Playing TicTacToe with Baxter.
This code is released under a GNU GPL v2 license (see LICENSE file for more information).

## Requirements

 * a Baxter robot
 * a usb camera

## Usage

   # How to run this demo : a step by step tutorial

    ## Initialization

      * Obvious stuff:
        * make sure the Baxter is on
        * make sure the computer is on
        * make sure the Baxter and the computer are connected together
        * make sure the computer has some working speakers

    ## Calibration 
      * Open a new terminal
        * `cd` to `~/ros_ws`
        * `source devel/setup.bash`
        * `./baxter.sh` (this has to be done with any terminal that will interface with the Baxter)
        * `rosrun baxter_tools tuck_arms.py -u`
        * Manually move right arm outside camera's field of view
        * `roslaunch baxter_tictactoe tictactoe.launch`
        * Move board to correct position via observation of right arm movement
        * Exit program

      * Open a terminal:
        * 'roslaunch ttt_board_sensor cells_definition launch'. Select board area and save in '~/ros_devel_ws/src/baxter_tictactoe/ttt_board_sensor/resources' as 'ttt_board_lab.xml'
        * 'roslaunch ttt_board_sensor display_board_cells.launch'
        * 'roslaunch ttt_board_sensor board_state_sensor.launch show:=true'

    ## Run the demo
      * Open a new terminal
        * `cd` to `~/ros_ws`
        * `source devel/setup.bash`
        * `./baxter.sh` (this has to be done with any terminal that will interface with the Baxter)
        * `roslaunch baxter_tictactoe tictactoe.launch`    

    ## Shut down the robot
      * Open a terminal:
        * `cd` to `~/ros_ws`
        * `source devel/setup.bash`
        * `./baxter.sh` (this has to be done with any terminal that will interface with the Baxter)
        * `rosrun baxter_tools tuck_arms.py -t`


