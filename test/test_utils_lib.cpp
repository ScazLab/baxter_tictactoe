#include <gtest/gtest.h>

#include "baxter_tictactoe/tictactoe_utils.h"

using namespace ttt;

// Declare a test
TEST(UtilsLib, testCellClass)
{
    Cell cell_a;

    EXPECT_EQ(cell_a.getState(), COL_EMPTY);

    EXPECT_EQ(cell_a.setState(COL_RED), true);
    EXPECT_EQ(cell_a.getState(), COL_RED);

    EXPECT_EQ(cell_a.setState(COL_BLUE), true);
    EXPECT_EQ(cell_a.getState(), COL_BLUE);

    EXPECT_EQ(cell_a.setState("foo"), false);

    EXPECT_EQ(cell_a. getRedArea(), 0);
    EXPECT_EQ(cell_a.getBlueArea(), 0);

    cell_a. setRedArea(10);
    cell_a.setBlueArea(20);
    EXPECT_EQ(cell_a.  getRedArea(), 10);
    EXPECT_EQ(cell_a. getBlueArea(), 20);

    EXPECT_EQ(cell_a.computeState(), true);
    EXPECT_EQ(cell_a.getState(), COL_BLUE);

    EXPECT_EQ(cell_a. resetState(),      true);
    EXPECT_EQ(cell_a.   getState(), COL_EMPTY);
    EXPECT_EQ(cell_a. getRedArea(), 0);
    EXPECT_EQ(cell_a.getBlueArea(), 0);
}

TEST(UtilsLib, testBoardClass)
{

    EXPECT_TRUE(true);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
