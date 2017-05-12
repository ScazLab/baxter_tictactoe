# Baxter TicTacToe [![Build Status](https://travis-ci.org/ScazLab/baxter_tictactoe.svg?branch=master)](https://travis-ci.org/ScazLab/baxter_tictactoe) [![Issues](https://img.shields.io/github/issues/ScazLab/baxter_tictactoe.svg?label=Issues)](https://github.com/ScazLab/baxter_tictactoe/issues) [![Codacy Badge](https://api.codacy.com/project/badge/Grade/0d9745ef739e4a399abb58b025d9fc19)](https://www.codacy.com/app/Baxter-collaboration/baxter_tictactoe?utm_source=github.com&amp;utm_medium=referral&amp;utm_content=ScazLab/baxter_tictactoe&amp;utm_campaign=Badge_Grade)

Playing TicTacToe with the Baxter collaborative robot.

![setup](https://cloud.githubusercontent.com/assets/4378663/26005307/ac35d298-3706-11e7-84c7-c278a01fe3b7.jpg)

This code is released under a GNU GPL v2 license (see LICENSE file for more information).

## Requirements

 * a Baxter robot
 * a usb camera

## Usage

### Initialization

Obvious stuff:

 * make sure the Baxter is on
 * make sure the computer is on
 * make sure the Baxter and the computer are connected together
 * make sure the computer has some working speakers
 * make sure the board and tokens are placed on a table in front of Baxter

### Calibration

In order to calibrate the hard coded positions for the pile of objects and the board, just record the end-effector position of the left arm when positioned either on the pile of tiles, or the four corners of the board. To to so, the following command may be useful:

```
rostopic echo -n 1 /robot/limb/right/endpoint_state | grep -A 3 position
```

Then, use this information to populate the corresponding parameters in `tictactoe.launch` : `"ttt_controller/tile_pile_position` and `ttt_controller/board_corner_poss`.

### Run the demo

 * Make sure the board is empty
 * Open a new terminal
 * `cd` to `~/ros_ws`
 * `source devel/setup.bash`
 * `./baxter.sh` (this has to be done with any terminal that will interface with the Baxter)
 * `rosrun baxter_tools tuck_arms.py -u`
 * `roslaunch baxter_tictactoe tictactoe.launch`
 * Exit program

### Shut down the robot

 * Open a terminal:
 * `cd` to `~/ros_ws`
 * `source devel/setup.bash`
 * `./baxter.sh` (this has to be done with any terminal that will interface with the Baxter)
 * `rosrun baxter_tools tuck_arms.py -t`


