#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <tictactoe/PlaceTokenAction.h>

int main (int argc, char **argv)
{
    ros::init(argc, argv, "test_move_maker");

    // true causes the client to spin its own thread
    actionlib::SimpleActionClient<tictactoe::PlaceTokenAction> move_commander("place_token", true);

    ROS_INFO("Waiting for TTT Move Maker action server to start.");
    ROS_ASSERT_MSG(move_commander.waitForServer(ros::Duration(10.0)),"TTT Move Maker action server doesn't found");
    ROS_INFO("TTT Move Maker action server is started. We are ready for sending goals.");

    tictactoe::PlaceTokenGoal goal;
    int i=0;
    while(true){
        ROS_INFO("Choose the cell to place the next token [1..9], or other value to exit.");
        ROS_INFO(" ___________");
        ROS_INFO("| 1 | 2 | 3 |");
        ROS_INFO("|---|---|---|");
        ROS_INFO("| 4 | 5 | 6 |");
        ROS_INFO("|---|---|---|");
        ROS_INFO("|_7_|_8_|_9_|");
        std::cin >> i;
        switch(i){
        case 1:goal.cell="1x1"; break;
        case 2:goal.cell="1x2"; break;
        case 3:goal.cell="1x3"; break;
        case 4:goal.cell="2x1"; break;
        case 5:goal.cell="2x2"; break;
        case 6:goal.cell="2x3"; break;
        case 7:goal.cell="3x1"; break;
        case 8:goal.cell="3x2"; break;
        case 9:goal.cell="3x3"; break;
        default: return 0;//ros::shutdown();
        }
        move_commander.sendGoal(goal);
        bool finished_before_timeout = move_commander.waitForResult(ros::Duration(40.0)); //wait 40s for the action to return
        if (finished_before_timeout) ROS_INFO_STREAM("Action moving to " << goal.cell << " finished!");
        else ROS_INFO_STREAM("Action moving to " << goal.cell << " did not finish before the time out.");
        actionlib::SimpleClientGoalState state = move_commander.getState();
        ROS_INFO("State after the action: %s",state.toString().c_str());
    }
}

