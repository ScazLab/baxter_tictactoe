PACKAGE = 'baxter_tictactoe'

import rospy
import roslib
import std_msgs.msg
roslib.load_manifest(PACKAGE)

from baxter_tictactoe.msg import ttt_board as BoardMsg

rospy.init_node('fake_board_sensor')


EMPTY = BoardMsg.EMPTY
BLUE = BoardMsg.BLUE
RED = BoardMsg.RED
SYMBOLS = {BLUE: 'X', RED: 'O', EMPTY: ' '}
X = BLUE
O = RED

pub = rospy.Publisher("baxter_tictactoe/board_state", BoardMsg)


class Board:

    hdr = std_msgs.msg.Header()

    def __init__(self):
        self.clear()

    def __repr__(self):
        line = '|{} {} {}|\n'
        s = ''
        for i in range(3):
            values = [self.data[j] for j in range(3 * i, 3 * (i + 1))]
            s += line.format(*[SYMBOLS[v] for v in values])
        return s

    def __lt__(self, other):
        """Set a value to cell.

            >>> b = Board()
            >>> b < (2, X)
            >>> | |X| |
            >>> | | | |
            >>> | | | |
        """
        c, v = other
        self.set(c, v)

    def __le__(self, v):
        self.set(v, self.pop_turn())

    __lshift__ = __le__

    def pop_turn(self):
        if self._turn == X:
            self._turn = O
            return X
        else:
            self._turn = X
            return O

    def set(self, cell, value):
        self.data[cell - 1] = value
        self.send()
        print(self)
        print('Next: {}'.format(SYMBOLS[self._turn]))

    def send(self):
        self.hdr.stamp = rospy.Time.now()
        pub.publish(self.hdr, self.data)

    def clear(self):
        self.data = [EMPTY for _dummy in range(9)]
        self._turn = X  # X always starts
        self.send()


b = Board()
