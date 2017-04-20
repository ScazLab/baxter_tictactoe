#include <gtest/gtest.h>

#include "baxter_tictactoe/tictactoe_utils.h"

using namespace ttt;

// Declare a test
TEST(UtilsLib, testCellClass)
{
    // Testing default constructor
    Cell a;

    EXPECT_EQ(a.getState(), COL_EMPTY);

    EXPECT_EQ(a.setState(COL_RED), true);
    EXPECT_EQ(a.getState(), COL_RED);

    EXPECT_EQ(a.setState(COL_BLUE), true);
    EXPECT_EQ(a.getState(), COL_BLUE);

    EXPECT_EQ(a.setState("foo"), false);

    EXPECT_EQ(a. getRedArea(), 0);
    EXPECT_EQ(a.getBlueArea(), 0);

    a. setRedArea(10);
    a.setBlueArea(20);
    EXPECT_EQ(a.  getRedArea(), 10);
    EXPECT_EQ(a. getBlueArea(), 20);

    EXPECT_EQ(a.computeState(), true);
    EXPECT_EQ(a.getState(), COL_BLUE);

    // Testing copy constructor
    Cell b(a);
    EXPECT_EQ(b.  getRedArea(), 10);
    EXPECT_EQ(b. getBlueArea(), 20);

    EXPECT_EQ(b.computeState(), true);
    EXPECT_EQ(b.getState(), COL_BLUE);

    // Testing comparison operators
    EXPECT_EQ(b == a,  true);
    EXPECT_EQ(a == b,  true);
    EXPECT_EQ(b != a, false);
    EXPECT_EQ(a != b, false);

    // Testing assignment operator
    Cell c;
    c = b;
    EXPECT_EQ(c.  getRedArea(), 10);
    EXPECT_EQ(c. getBlueArea(), 20);

    EXPECT_EQ(c.computeState(), true);
    EXPECT_EQ(c.getState(), COL_BLUE);

    // Testing comparison operators
    EXPECT_EQ(c == a,  true);
    EXPECT_EQ(a == c,  true);
    EXPECT_EQ(c != a, false);
    EXPECT_EQ(a != c, false);

    // Testing resetState
    EXPECT_EQ(a. resetState(),      true);
    EXPECT_EQ(a.   getState(), COL_EMPTY);
    EXPECT_EQ(a. getRedArea(), 0);
    EXPECT_EQ(a.getBlueArea(), 0);

    // Testing comparison operators
    EXPECT_EQ(b == a, false);
    EXPECT_EQ(a == b, false);
    EXPECT_EQ(b != a,  true);
    EXPECT_EQ(a != b,  true);
    EXPECT_EQ(c == a, false);
    EXPECT_EQ(a == c, false);
    EXPECT_EQ(c != a,  true);
    EXPECT_EQ(a != c,  true);

    // Testing setState
    a.setState(COL_EMPTY);
    b.setState(COL_BLUE);
    c.setState(COL_RED);

    a.computeState();
    EXPECT_EQ(a.getState(), COL_EMPTY);

    b.computeState();
    EXPECT_EQ(b.getState(), COL_BLUE);
    b.setState(COL_RED);
    b.computeState();
    EXPECT_EQ(b.getState(), COL_RED);

    c.computeState();
    EXPECT_EQ(c.getState(), COL_RED);
    c.setState(COL_BLUE);
    c.computeState();
    EXPECT_EQ(c.getState(), COL_BLUE);
}

TEST(UtilsLib, testBoardClass)
{
    // Testing empty constructor
    Board a;

    // An empty board is both empty and full at the same time,
    // because there are no cells in check against
    EXPECT_TRUE(a.isEmpty());
    EXPECT_TRUE(a.isFull());

    // Testing constructor with number of cells
    Board b(9);

    EXPECT_TRUE(b.isEmpty());
    EXPECT_FALSE(b.isFull());
    EXPECT_EQ(b.getNumCells(), 9);

    // Testing toMsgBoard/fromMsgBoard
    a.fromMsgBoard(b.toMsgBoard());
    EXPECT_EQ(a, b);

    // Testing isEqual/isDifferent
    EXPECT_TRUE (a == b);
    EXPECT_FALSE(a != b);

    // Testing copy constructor
    Board c(a);

    EXPECT_TRUE (a == c);
    EXPECT_FALSE(a != c);

    // Start assigning random values to the boards
    for (size_t i = 0; i < b.getNumCells(); ++i)
    {
        b.setCell(i, Cell(COL_RED));
    }

    EXPECT_EQ(b.getNumTokens(),        b.getNumCells());
    EXPECT_EQ(b.getNumTokens(COL_RED), b.getNumCells());

    EXPECT_FALSE(b.isEmpty());
    EXPECT_TRUE(b.isFull());

    b.setCellState(4, COL_EMPTY);
    EXPECT_FALSE(b.isEmpty());
    EXPECT_FALSE(b.isFull());

    // Testing toMsgBoard/fromMsgBoard
    a.fromMsgBoard(b.toMsgBoard());
    EXPECT_EQ(a, b);

    // Testing resetCellStates
    b.resetCellStates();
    EXPECT_TRUE(b.isEmpty());
    EXPECT_FALSE(b.isFull());

    // Testing resetBoard
    b.resetBoard();
    // An empty board is both empty and full at the same time,
    // because there are no cells in check against
    EXPECT_TRUE(b.isEmpty());
    EXPECT_TRUE(b.isFull());
}

TEST(UtilsLib, testBoardStates)
{
    Board a(9);

    a.setCellState(0, COL_RED);
    a.setCellState(1, COL_RED);
    a.setCellState(2, COL_RED);
    EXPECT_TRUE(a.threeInARow());
    EXPECT_TRUE(a.threeInARow(COL_RED));

    a.setCellState(1, COL_BLUE);
    EXPECT_FALSE(a.threeInARow());
    EXPECT_FALSE(a.threeInARow(COL_RED));
    EXPECT_FALSE(a.threeInARow(COL_BLUE));

    a.setCellState(4, COL_BLUE);
    a.setCellState(7, COL_BLUE);
    EXPECT_TRUE(a.threeInARow());
    EXPECT_TRUE(a.threeInARow(COL_BLUE));

    a.setCellState(3, COL_RED);
    a.setCellState(6, COL_RED);
    EXPECT_TRUE(a.threeInARow());
    EXPECT_TRUE(a.threeInARow(COL_RED));
    EXPECT_TRUE(a.threeInARow(COL_BLUE));

    a.setCellState(4, COL_RED);
    a.setCellState(3, COL_EMPTY);
    EXPECT_TRUE(a.threeInARow());
    EXPECT_TRUE(a.threeInARow(COL_RED));
    EXPECT_FALSE(a.threeInARow(COL_BLUE));

    a.setCellState(8, COL_RED);
    a.setCellState(6, COL_EMPTY);
    EXPECT_TRUE(a.threeInARow());
    EXPECT_TRUE(a.threeInARow(COL_RED));
    EXPECT_FALSE(a.threeInARow(COL_BLUE));
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
