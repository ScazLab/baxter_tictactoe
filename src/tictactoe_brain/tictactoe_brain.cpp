#include "tictactoeBrain.h"

#define NUM_GAMES           3
#define CHEATING_GAME_A     2
#define CHEATING_GAME_B     3

using namespace ttt;

int main(int argc, char** argv)
{    
    ROS_INFO("[tictactoeBrain] Playing TIC TAC TOE");
    ros::init(argc, argv, "ttt_brain");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ttt::tictactoeBrain brain; //random strategy by default
    brain.set_strategy("smart");
    ros::Duration(1).sleep(); //this second is needed in order to use the voice at the beggining
    ROS_INFO_STREAM("[tictactoeBrain] Robot plays with " << brain.get_robot_color_str() << " and the opponent with " << brain.get_opponent_color_str());

    ROS_WARN("[tictactoeBrain] PRESS ENTER TO START WITH A NEW PARTICIPANT");
    std::cin.get();
    brain.say_sentence("  Welcome!  Let's play Tic Tac Toe.",4);
    brain.say_sentence("Do not grasp your token before I say that it is your turn",5);

    uint robot_victories=0, participant_victories=0, ties=0;
    uint i=1;
    bool cheating=false;
    ROS_INFO_STREAM("[tictactoeBrain] Let's play " << NUM_GAMES << " times Tic Tac Toe");
    unsigned short game_result=0;

    while(i<=NUM_GAMES)
    {
        ROS_INFO_STREAM("[tictactoeBrain] Game " << i);

        if ((i==CHEATING_GAME_A || i==CHEATING_GAME_B) && brain.get_cheating()) //In the fourth game, Baxter cheats
        {
            brain.set_strategy("smart-cheating");
        }

        game_result=brain.play_one_game(cheating);

        switch(game_result)
        {
        case 1: robot_victories++;
            break;
        case 2: participant_victories++;
            break;
        case 3: ties++;
            break;
        default: ROS_ERROR_STREAM("[tictactoeBrain] Unexpected return value for the game: " << game_result << " ???");
        }

        if ((i==CHEATING_GAME_A || i==CHEATING_GAME_B) && brain.get_cheating())
        {
            if (!cheating)
            {
                ROS_INFO("[tictactoeBrain] Game ended but no cheating. Game counter does not increase.");
                continue;
            }
            else
            {
                ROS_INFO("[tictactoeBrain] Game ended cheating. Back to random strategy");
                brain.set_strategy("smart");
            }
        }
        i++;
    }

    brain.say_sentence("Game over. It was my pleasure to win over you. Thanks for being so human.",10);

    ROS_INFO_STREAM("[tictactoeBrain] Baxter " << robot_victories << " - Human " << participant_victories << " - Ties " << ties);

    return 0;
}
