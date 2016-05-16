# Message with the info of the tictactoe board

Header header

# Information from each cell in the board
# Index of each cell in the array:
#            _ _ _ _ _ _
#           |_0_|_1_|_2_|
#           |_3_|_4_|_5_|
#           |_6_|_7_|_8_|
#
# Possible values:
#       0=empty cell
#       1=blue token (X)
#       2=red token (O)

uint8 EMPTY=0
uint8 BLUE=1
uint8 RED=2
uint8 UNDEFINED=3

uint8[9] data
