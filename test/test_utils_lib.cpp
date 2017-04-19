#include <gtest/gtest.h>

#include "baxter_tictactoe/tictactoe_utils.h"

using namespace ttt;

// Declare a test
TEST(UtilsLib, testCellClass)
{
    // Testing default constructor
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

    // Testing copy constructor
    Cell cell_b(cell_a);
    EXPECT_EQ(cell_b.  getRedArea(), 10);
    EXPECT_EQ(cell_b. getBlueArea(), 20);

    EXPECT_EQ(cell_b.computeState(), true);
    EXPECT_EQ(cell_b.getState(), COL_BLUE);

    // Testing comparison operators
    EXPECT_EQ(cell_b == cell_a,  true);
    EXPECT_EQ(cell_a == cell_b,  true);
    EXPECT_EQ(cell_b != cell_a, false);
    EXPECT_EQ(cell_a != cell_b, false);

    // Testing assignment operator
    Cell cell_c;
    cell_c = cell_b;
    EXPECT_EQ(cell_c.  getRedArea(), 10);
    EXPECT_EQ(cell_c. getBlueArea(), 20);

    EXPECT_EQ(cell_c.computeState(), true);
    EXPECT_EQ(cell_c.getState(), COL_BLUE);

    // Testing comparison operators
    EXPECT_EQ(cell_c == cell_a,  true);
    EXPECT_EQ(cell_a == cell_c,  true);
    EXPECT_EQ(cell_c != cell_a, false);
    EXPECT_EQ(cell_a != cell_c, false);

    // Testing resetState
    EXPECT_EQ(cell_a. resetState(),      true);
    EXPECT_EQ(cell_a.   getState(), COL_EMPTY);
    EXPECT_EQ(cell_a. getRedArea(), 0);
    EXPECT_EQ(cell_a.getBlueArea(), 0);

    // Testing comparison operators
    EXPECT_EQ(cell_b == cell_a, false);
    EXPECT_EQ(cell_a == cell_b, false);
    EXPECT_EQ(cell_b != cell_a,  true);
    EXPECT_EQ(cell_a != cell_b,  true);
    EXPECT_EQ(cell_c == cell_a, false);
    EXPECT_EQ(cell_a == cell_c, false);
    EXPECT_EQ(cell_c != cell_a,  true);
    EXPECT_EQ(cell_a != cell_c,  true);
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
