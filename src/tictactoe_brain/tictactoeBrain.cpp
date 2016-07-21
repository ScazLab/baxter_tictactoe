#include "tictactoeBrain.h"

using namespace ttt;
using namespace std;
using namespace baxter_tictactoe;

bool ttt::operator==(boost::array<baxter_tictactoe::MsgCell, NUMBER_OF_CELLS> cells1, boost::array<baxter_tictactoe::MsgCell, NUMBER_OF_CELLS> cells2)
{
    for(int i = 0; i < cells1.size(); i++){
        if(cells1[i].state != cells2[i].state){
            return false;
        }
    }
    return true;
}

bool ttt::operator!=(boost::array<baxter_tictactoe::MsgCell, NUMBER_OF_CELLS> cells1, boost::array<baxter_tictactoe::MsgCell, NUMBER_OF_CELLS> cells2)
{
    return !(cells1==cells2);
}

// traj=false -> arm movement done via inverse kinematics (ArmController)
// traj=true -> arm movement done via a joint trajectory action server (MoveMaker, MoveMakerServer, TrajectoryPlayer)
tictactoeBrain::tictactoeBrain(bool traj, cellState robot_color, std::string strategy) : _robot_color(robot_color), traj(traj), _setup(false), _move_commander("place_token", true) // true causes the client to spin its own thread
{
    ROS_DEBUG("[tictactoeBrain] traj = %d", traj);
    if(traj == false)
    {
        _left_ac = new ArmController("left");
        _right_ac = new ArmController("right");

        std::string left = "left";
        std::string right = "right";
        int failure;
        void *status;

        ROS_INFO("Moving arms to rest..");
        _left_ac->moveToRest();
        _right_ac->moveToRest();
        while(!(_left_ac->getState() == REST && _right_ac->getState() == REST)) {ros::spinOnce();}

        ROS_INFO("Scanning board with left arm..");
        _left_ac->scanBoard();
        while(_left_ac->getState() != SCAN){ros::spinOnce();}
    }

    _number_of_tokens_on_board.set(0);

    ROS_ASSERT_MSG(_nh.hasParam("voice"),"No voice found in the parameter server!");
    ROS_ASSERT_MSG(_nh.getParam("voice",_voice_type), "The voice parameter not retrieve from the parameter server");
    ROS_INFO_STREAM("[tictactoeBrain] Using voice " << _voice_type);

    ROS_ASSERT_MSG(_nh.hasParam("cheating"),"No cheating parameter found in the parameter server!");
    ROS_ASSERT_MSG(_nh.getParam("cheating",cheating), "The cheating possibility has not been retrieved from the parameter server");
    ROS_INFO_STREAM("[tictactoeBrain] Robot " << (cheating?"can":"cannot") << " cheat");

    ROS_ASSERT_MSG(_nh.hasParam("smooth"),"No sort of movements found in the parameter server!");
    ROS_ASSERT_MSG(_nh.getParam("smooth",movement_type), "The sort of movements has not been retrieved from the parameter server");
    ROS_INFO_STREAM("[tictactoeBrain] Using " << movement_type << " movements");
   
    // if(traj == true)
    // {
    //     _clnt_movement_type = _nh.serviceClient<baxter_tictactoe::SetTrajectoryType>("set_movement_type");
    //     set_movement_type(movement_type);    
    // }

    ROS_ASSERT_MSG(_robot_color==blue || _robot_color==red, "Wrong color for robot's tokens");
    _opponent_color=_robot_color==blue?red:blue;

    TTT_State_type aux; // aux is an array of 9 MsgCells 
    for(int i = 0; i < aux.size(); i++)
    {
        aux[i].state = baxter_tictactoe::MsgCell::UNDEFINED;
    }

    // aux.assign(undefined);
    _ttt_state.set(aux);    // initialy the state for all cells in the TTT board is undefined

    _ttt_state_sub = _nh.subscribe("baxter_tictactoe/new_board", 1, &tictactoeBrain::new_ttt_state, this); //receiving data about the TTT board state every time it changes        
    _scan_server = _nh.advertiseService("baxter_tictactoe/ready_scan", &tictactoeBrain::scanState, this);

    qsrand(ros::Time::now().nsec);
    set_strategy(strategy);

    has_cheated=false;

    if(traj == true)
    {
        ROS_INFO("[tictactoeBrain] Waiting for TTT Move Maker action server to start.");
        ROS_ASSERT_MSG(_move_commander.waitForServer(ros::Duration(10.0)),"TTT Move Maker action server doesn't found");
        ROS_INFO("[tictactoeBrain] TTT Move Maker action server is started. We are ready for sending goals.");       
    }

    _setup = true; // indicates whether board has been scanned by arm camera 
                   // (signal to boardStateSensing that the board's position 
                   // is locked and ready to scanned by head camera)

    while(ros::ok())
    {
        TTT_State_type arr = _ttt_state.get();       
        if(arr != aux) 
        {
            break;
        }
        else 
        {
            ROS_WARN("Board was not detected. Make sure board is within the head camera's view and is not obscured. Press ENTER afterwards.");
            char c = cin.get();
        }
    }
}

tictactoeBrain::~tictactoeBrain()
{
    delete _left_ac;
    delete _right_ac;
}

bool tictactoeBrain::scanState(ScanState::Request &req, ScanState::Response &res)
{
    res.state = _setup == true ? true : false;
    return true;
}

void tictactoeBrain::new_ttt_state(const baxter_tictactoe::MsgBoardConstPtr & msg)
{
    if ((msg->cells)!=_ttt_state.get()) {
        ROS_DEBUG_STREAM("[tictactoeBrain] New TTT board state detected at ." << msg->header.stamp);
        _ttt_state.set(msg->cells);
    }
    else{
        ROS_DEBUG("[tictactoeBrain] Same board received. Do not update.");
    }
}

int tictactoeBrain::random_move(bool& cheating)
{
    cheating=false;
    int r;
    TTT_State_type aux = _ttt_state.get();
    do {
        r = qrand() % NUMBER_OF_CELLS + 1; //random number between 1 and NUMBER_OF_CELLS
        ROS_DEBUG("[tictactoeBrain] Cell %d is in state %d ==? %d || %d", r, aux[r-1].state, 
                  baxter_tictactoe::MsgCell::EMPTY, baxter_tictactoe::MsgCell::UNDEFINED);
    }
    while(aux[r-1].state!=empty && aux[r-1].state!=undefined);

    ROS_INFO_STREAM("[tictactoeBrain] Random move to cell with number " << r);
    return r;
}

int tictactoeBrain::cheating_to_win_random_move(bool& cheating)
{
    int next_cell_id=-1;
    if ((next_cell_id = victory_move()) != -1) return next_cell_id;
    if ((next_cell_id = cheating_move()) != -1)
    {
        cheating=true;
        return next_cell_id;
    }
    if ((next_cell_id = defensive_move()) != -1) return next_cell_id;
    return random_move(cheating);
}

int tictactoeBrain::winning_defensive_random_move(bool& cheating)
{
    cheating=false;
    int next_cell_id=-1;
    if ((next_cell_id = victory_move()) != -1) return next_cell_id;
    if ((next_cell_id = defensive_move()) != -1) return next_cell_id;
    return random_move(cheating);
}

int tictactoeBrain::smart_cheating_random_move(bool& cheating)
{
    int next_cell_id=-1;
    if ((next_cell_id = victory_move()) != -1) return next_cell_id;
    if ((next_cell_id = cheating_move()) != -1)
    {
        cheating=true;
        return next_cell_id;
    }
    if ((next_cell_id = defensive_move()) != -1) return next_cell_id;
    return random_move(cheating);
}

int tictactoeBrain::cheating_move()
{
    TTT_State_type aux = _ttt_state.get();
    uint8_t cell_state = undefined;
    for (size_t i = 0; i < NUMBER_OF_CELLS; ++i) {
        if (aux[i].state==_opponent_color)
        {
            cell_state=aux[i].state;
            aux[i].state=_robot_color;
            if (tictactoeBrain::three_in_a_row(_robot_color, aux))
            {
                ROS_INFO_STREAM("[tictactoeBrain] Cheating move to cell with number " << i+1);
                has_cheated=true;
                return i+1;
            }
            aux[i].state=cell_state;
        }
    }
    ROS_INFO("[tictactoeBrain] No chance to win in this turn");
    return -1;
}

int tictactoeBrain::defensive_move()
{
    TTT_State_type aux = _ttt_state.get();
    uint8_t cell_state = undefined;
    for (size_t i = 0; i < NUMBER_OF_CELLS; ++i) {
        if (aux[i].state==empty || aux[i].state==undefined)
        {
            cell_state = aux[i].state;
            aux[i].state = _opponent_color;
            if (tictactoeBrain::three_in_a_row(_opponent_color, aux))
            {
                ROS_INFO_STREAM("[tictactoeBrain] Defensive move to cell with number " << i+1);
                return i+1;
            }
            aux[i].state=cell_state;
        }
    }
    ROS_INFO("[tictactoeBrain] No chance to lose in this turn");
    return -1;
}

int tictactoeBrain::victory_move()
{
    TTT_State_type aux = _ttt_state.get();
    uint8_t cell_state = undefined;
    
    for (size_t i = 0; i < NUMBER_OF_CELLS; ++i)
    {
        if (aux[i].state==empty || aux[i].state==undefined)
        {
            cell_state = aux[i].state;
            aux[i].state = _robot_color;
            if (tictactoeBrain::three_in_a_row(_robot_color, aux))
            {
                ROS_INFO_STREAM("[tictactoeBrain] Victory move to cell with number " << i+1);
                return i+1;
            }
            aux[i].state = cell_state;
        }
    }

    ROS_INFO("[tictactoeBrain] No chance to win in this turn");
    return -1;
}

// bool tictactoeBrain::set_movement_type(bool b)
// {
//     movement_type=b;
//     baxter_tictactoe::SetTrajectoryType srv;
//     srv.request.smooth=(movement_type?true:false);

//     if(_clnt_movement_type.call(srv))
//     {
//         if (!srv.response.error)
//         {
//             ROS_INFO_STREAM("[tictactoeBrain] Movements set to " << (srv.request.smooth?"smooth":"mechanistic"));
//             return true;
//         }
//         ROS_ERROR_STREAM("[tictactoeBrain] Error setting movements to " << (srv.request.smooth?"smooth":"mechanistic"));
//         return false;
//     }
//     ROS_ERROR("[tictactoeBrain] Failed to call service set_movement_type");
//     return false;
// }

int tictactoeBrain::get_next_move(bool& cheating)
{
    return (this->*_choose_next_move)(cheating);
}

bool tictactoeBrain::ismovement_type_movements()
{
    return movement_type;
}

actionlib::SimpleClientGoalState tictactoeBrain::execute_move(int cell_to_move)
{
    tictactoe::PlaceTokenGoal goal;
    switch(cell_to_move)
    {
        case 1:goal.cell="1x1"; break;
        case 2:goal.cell="1x2"; break;
        case 3:goal.cell="1x3"; break;
        case 4:goal.cell="2x1"; break;
        case 5:goal.cell="2x2"; break;
        case 6:goal.cell="2x3"; break;
        case 7:goal.cell="3x1"; break;
        case 8:goal.cell="3x2"; break;
        case 9:goal.cell="3x3"; break;
        default:
            ROS_ERROR("[tictactoeBrain] Unknown cell to move, %d",cell_to_move);
            return actionlib::SimpleClientGoalState::LOST;
    }
    
    _move_commander.sendGoal(goal);
    _move_commander.waitForResult(ros::Duration(40.0)); //wait 40s for the action to return
    bool _success = (_move_commander.getState() == actionlib::SimpleClientGoalState::SUCCEEDED);
    //bool finished_before_timeout = _move_commander.waitForResult(ros::Duration(40.0)); //wait 40s for the action to return
    //if (finished_before_timeout) ROS_INFO_STREAM("[tictactoeBrain] Action moving to " << goal.cell << " finished successfully!");
    if (_success) ROS_INFO_STREAM("[tictactoeBrain] Action moving to " << goal.cell << " finished successfully!");
    else ROS_WARN_STREAM("[tictactoeBrain] Action moving to " << goal.cell << " did not finish before the time out.");
    return _move_commander.getState();
}

unsigned short int tictactoeBrain::get_number_of_tokens_on_board()
{
    unsigned short int counter=0;
    TTT_State_type aux = _ttt_state.get();
    for(int i = 0; i < aux.size(); i++)
    {
        if(aux[i].state!=empty && aux[i].state!=undefined) counter++;
    }
    return counter;
}

unsigned short int tictactoeBrain::get_number_of_tokens_on_board(cellState token_type)
{
    unsigned short int counter=0;
    TTT_State_type aux = _ttt_state.get();
    for(int i = 0; i < aux.size(); i++)
    {
       if(aux[i].state==token_type) counter++;
    }
    return counter;
}

bool tictactoeBrain::three_in_a_row(const cellState& cell_color, const TTT_State_type& tttboard)
{
    ROS_DEBUG("[tictactoeBrain] @three in a row");
    if(cell_color!=blue && cell_color!=red) return false; // There are only red and blue tokens

    if(tttboard[0].state==cell_color && tttboard[1].state==cell_color && tttboard[2].state==cell_color) return true; // 3 in a row in the first row
    if(tttboard[3].state==cell_color && tttboard[4].state==cell_color && tttboard[5].state==cell_color) return true; // 3 in a row in the second row
    if(tttboard[6].state==cell_color && tttboard[7].state==cell_color && tttboard[8].state==cell_color) return true; // 3 in a row in the third row
    if(tttboard[0].state==cell_color && tttboard[3].state==cell_color && tttboard[6].state==cell_color) return true; // 3 in a row in the first column
    if(tttboard[1].state==cell_color && tttboard[4].state==cell_color && tttboard[7].state==cell_color) return true; // 3 in a row in the second column
    if(tttboard[2].state==cell_color && tttboard[5].state==cell_color && tttboard[8].state==cell_color) return true; // 3 in a row in the third column
    if(tttboard[0].state==cell_color && tttboard[4].state==cell_color && tttboard[8].state==cell_color) return true; // 3 in a row in the first diagonal
    if(tttboard[2].state==cell_color && tttboard[4].state==cell_color && tttboard[6].state==cell_color) return true; // 3 in a row in the second diagonal

    return false;
}

bool tictactoeBrain::three_in_a_row(const cellState& cell_color)
{
    return three_in_a_row(cell_color,_ttt_state.get());
}

unsigned short int tictactoeBrain::get_winner()
{
    if (three_in_a_row(_robot_color,_ttt_state.get())) return 1;
    if (three_in_a_row(_opponent_color, _ttt_state.get())) return 2;
    return 0;
}

void tictactoeBrain::wait_for_opponent_turn(const uint8_t& n_opponent_tokens)
{        
    uint8_t aux_n_opponent_tokens=n_opponent_tokens;
    while(aux_n_opponent_tokens<=n_opponent_tokens) /* Waiting for my turn: the participant 
                                                       has to place one token, so we wait until
                                                       the number opponent's tokens increase. */
    {
        // ros::Duration(1).sleep();
        ROS_WARN("[tictactoeBrain] Press ENTER when the opponent's turn is done");
        std::cin.get();
        aux_n_opponent_tokens=get_number_of_tokens_on_board(_opponent_color);
    }
}

bool tictactoeBrain::is_board_full()
{
    TTT_State_type aux = _ttt_state.get();
    for(int i = 0; i < aux.size(); i++){
        if(aux[i].state==empty || aux[i].state==undefined) /* we consider the case where cells are 
                                                              undefined because is needed in the first
                                                              loop when all cells are undefined */
        return false;
    }
    return true;
}

void tictactoeBrain::say_sentence(std::string sentence, double t)
{
    _voice_synthesizer.say(sentence, _voice_type);
    if(_nh.ok()) ros::Duration(t).sleep();
    else ROS_WARN("[tictactoeBrain] Nodel handle is not ok. Don't sleeping during the synthetitation.");
}

void tictactoeBrain::set_strategy(std::string strategy)
{

    if (strategy=="random")
    {
        _choose_next_move=&tictactoeBrain::random_move;
        ROS_INFO("[tictactoeBrain] New strategy: Baxter randomly places tokens");
    } else if (strategy=="smart")
    {
        _choose_next_move=&tictactoeBrain::winning_defensive_random_move;
        ROS_INFO("[tictactoeBrain] New strategy: Baxter randomly places tokens but it wins, if there is a chance, or blocks opponent's victory");
    }
    else if (strategy=="cheating")
    {
        _choose_next_move=&tictactoeBrain::cheating_to_win_random_move;
        ROS_INFO("[tictactoeBrain] New strategy: Baxter randomly places tokens but it wins, if there is a chance even if cheating is required");
    }
    else if (strategy=="smart-cheating")
    {
        _choose_next_move=&tictactoeBrain::smart_cheating_random_move;
        ROS_INFO("[tictactoeBrain] New strategy: Baxter randomly places tokens but it wins, if there is a chance even if cheating is required, or blocks opponent's victory");
    }
    else {
        ROS_ERROR_STREAM(strategy << " is not an available strategy");
    }
}

unsigned short int tictactoeBrain::play_one_game(bool& cheating)
{

    bool robot_turn=true;
    unsigned short int winner=0; // no winner
    has_cheated=false;

    // say_sentence("Please place the blue tokens in the blue box on the right side of the board",6);
    // say_sentence(" and the red tokens in the red square on the left side of the board",6);
    say_sentence("I start the game.",2);

    ROS_WARN("[tictactoeBrain] PRESS ENTER TO START THE GAME");
    std::cin.get();
    uint8_t n_opponent_tokens=0;
    // uint8_t n_robot_tokens=0;
    while ((winner=get_winner())==0 && !is_board_full())
    {
        if (robot_turn) // Robot's turn
        {
            cout << "robot turn" << endl;

            // n_robot_tokens=get_number_of_tokens_on_board(_robot_color); //number of robot's tokens befor the robot's turn
            n_opponent_tokens=get_number_of_tokens_on_board(_opponent_color); //number of opponent's tokens befor the robot's turn
            say_sentence("It is my turn", 0.3);
            int cell_to_move = get_next_move(cheating);
            ROS_DEBUG_STREAM("[tictactoeBrain] Robot's token to " << cell_to_move);
            cout << "get next move" << endl;

            if(traj == false)
            {
                cout << "before arm movement" << endl;
                _left_ac->pickUpToken();
                while(_left_ac->getState() != PICK_UP)
                {
                    ros::spinOnce();
                }

                _left_ac->putDownToken(cell_to_move);
                while(_left_ac->getState() != PUT_DOWN){ros::spinOnce();}  
                cout << "after arm movement" << endl;
            }

            if(traj == true)
            {
                actionlib::SimpleClientGoalState goal_state = execute_move(cell_to_move);

                if (goal_state==actionlib::SimpleClientGoalState::SUCCEEDED)
                {
                    ROS_INFO("[tictactoeBrain] Last move successfully performed.");
                }
                else
                {
                    ROS_ERROR_STREAM("[tictactoeBrain] Last move has not succeded. Goal state " << goal_state.toString().c_str() << " is not guaranteed.");
                }            
            }

            //the number of robot tokens haven't increased so we try again.
            // if(n_robot_tokens==get_number_of_tokens_on_board(_robot_color)) 
            // {
            //     ROS_ERROR("[tictactoeBrain] Somehow there is still %ud robot tokens.",n_robot_tokens);
            //     continue;
            // }
        }
        else // Participant's turn
        {
            cout << "participant turn" << endl;

            ROS_INFO("[tictactoeBrain] Waiting for the participant's move.");
            say_sentence("It is your turn",0.1);
            wait_for_opponent_turn(n_opponent_tokens); /* Waiting for my turn: the participant 
                                                          has to place one token, so we wait 
                                                          until the number of the opponent's 
                                                          tokens increases. */
            cout << "after participant move" << endl;
        }
        robot_turn=!robot_turn;
    }

    switch(winner)
    {
    case 1:
        ROS_INFO("[tictactoeBrain] ROBOT's VICTORY!");
        if (has_cheated)
        {
            say_sentence("You humans are so easy to beat!",5);
        }
        say_sentence("I won", 3);
        break;
    case 2:
        ROS_INFO("[tictactoeBrain] OPPONENT's VICTORY!");
        say_sentence("You won this time",4);
        break;
    default:
        ROS_INFO("[tictactoeBrain] TIE!");
        say_sentence("That's a tie. I will win next time.",8);
        winner=3;
    }
    return winner;
}

