# Baxter TicTacToe [![Build Status](https://img.shields.io/travis/ScazLab/baxter_tictactoe/master.svg?label=Build Status)](https://travis-ci.org/ScazLab/baxter_tictactoe) [![Issues](https://img.shields.io/github/issues/ScazLab/baxter_tictactoe.svg?label=Issues)](https://github.com/ScazLab/baxter_tictactoe/issues)

Playing TicTacToe with the Baxter collaborative robot.
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


