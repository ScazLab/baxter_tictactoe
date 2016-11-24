#include "tictactoeBrain.h"

using namespace ttt;
using namespace std;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ttt_brain");
    ros::NodeHandle _nh("ttt_brain");

    int num_games;
    _nh.param<int>("num_games", num_games, NUM_GAMES);

    std::vector<int> cheating_games;
    if (_nh.hasParam("cheating_games"))
    {
        _nh.getParam("cheating_games", cheating_games);
    }
    else
    {
        cheating_games.push_back(CHEATING_GAME_A);
        cheating_games.push_back(CHEATING_GAME_B);
    }

    std::stringstream cheating_games_str;
    std::copy(cheating_games.begin(), cheating_games.end(),
              std::ostream_iterator<int>(cheating_games_str, " "));

    ROS_INFO("Number of games: %i; Cheating games: %s",
              num_games, cheating_games_str.str().c_str());

    ttt::tictactoeBrain brain("ttt_brain", "smart");

    ROS_INFO("Robot plays with %s tokens and the opponent with %s tokens.",
              brain.getRobotColor().c_str(), brain.getOpponentColor().c_str());

    // ros::spin();
    // return 0;

    brain.set_brain_state(baxter_tictactoe::TTTBrainState::MATCH_STARTED);
    brain.say_sentence("Welcome!  Let's play Tic Tac Toe.",4);
    brain.say_sentence("Do not grasp your token before I say that it is your turn",5);

    int n_robot_win=0;
    int n_opponent_win=0;
    int n_ties=0;

    int game_result=0;

    for (int i = 0; i < num_games; ++i)
    {
        printf("\n");
        ROS_INFO("GAME #%i", i);

        brain.set_strategy("smart");
        bool cheated=false;
        bool has_to_cheat=false;

        if (brain.get_cheating())
        {
            for (int j = 0; j < cheating_games.size(); ++j)
            {
                if (cheating_games[j] == i)
                {
                    brain.set_strategy("cheating");
                    has_to_cheat=true;
                }
            }
        }

        game_result=brain.play_one_game(cheated);

        switch(game_result)
        {
            case 1: n_robot_win++;
                break;
            case 2: n_opponent_win++;
                break;
            case 3: n_ties++;
                break;
            default: ROS_ERROR("Unexpected return value for the game: %i ???", game_result);
        }

        if (has_to_cheat && !cheated)
        {
            ROS_WARN("Game ended but no cheating. Game counter does not increase.");
            i--;
        }
    }

    brain.say_sentence("Game over. It was my pleasure to win over you. Thanks for being so human.",10);
    brain.set_brain_state(baxter_tictactoe::TTTBrainState::MATCH_FINISHED);

    ROS_INFO("Baxter %i - Human %i - n_ties %i", n_robot_win, n_opponent_win, n_ties);

    return 0;
}
