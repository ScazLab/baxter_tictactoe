#include "move_maker/move_maker.h"

using namespace ttt;

bool Move_Maker::get_ttt_trajectory(std::string traj_name, TTT_Trajectory& traj)
{
    ROS_DEBUG_STREAM("Looking for trajectory " << traj_name << " out of " << _trajectory_repository.size());
    if (_trajectory_repository.empty()) {
        ROS_WARN("Empty trajectory repository");
        return false;
    }
    foreach (TTT_Trajectory t, _trajectory_repository) {
        if(t.name==traj_name)
        {
            ROS_DEBUG_STREAM("Trajectory " << t.name << " exists with " << t.trajectory.points.size() << " points");
            traj=t;
            return true;
        }
    }
    return false;
}

bool Move_Maker::is_preempted()
{
    if (_place_token.isPreemptRequested()) {
        ROS_INFO_STREAM("place_token preempted");
        _place_token_result.success=false;
        _place_token.setPreempted(_place_token_result, "Action preempted");
        return true;
    }
    return false;
}

std::string *Move_Maker::get_trajectories_to_cell(std::string cell_id)
{
    if(cell_id=="1x1") return ttt::cell1x1;
    else if(cell_id=="1x2") return ttt::cell1x2;
    else if(cell_id=="1x3") return ttt::cell1x3;
    else if(cell_id=="2x1") return ttt::cell2x1;
    else if(cell_id=="2x2") return ttt::cell2x2;
    else if(cell_id=="2x3") return ttt::cell2x3;
    else if(cell_id=="3x1") return ttt::cell3x1;
    else if(cell_id=="3x2") return ttt::cell3x2;
    else if(cell_id=="3x3") return ttt::cell3x3;
    else return NULL;
}

bool Move_Maker::execute_single_trajectory(std::string traj_id, Trajectory_Type mode)
{
    TTT_Trajectory t;
    
    if(!this->get_ttt_trajectory(traj_id,t))
    {
        _place_token.setAborted(_place_token_result,traj_id + " trajectory not found");
        return false;
    }

    if (this->is_preempted()) return false;
    
    if (!ros::ok())
    {
        ROS_ERROR("ros node has been shut down");
        return false;
    }

    ROS_DEBUG_STREAM("[Move_Maker] Trajectory found. " << t.get_ttt_trajectory_description());
    bool success=true;
    switch(mode)
    {
    case PLAIN:
        ROS_INFO_STREAM("[Move_Maker] Playing PLAIN trajectory " << t.name);
        success=_traj_player->run_trajectory(t.trajectory);
        break;
    case GRASP:
        ROS_INFO_STREAM("[Move_Maker] Playing GRASP trajectory " << t.name);
        success=_traj_player->run_trajectory_and_grasp(t.trajectory);
        break;
    case RELEASE:
        ROS_INFO_STREAM("[Move_Maker] Playing RELEASE trajectory " << t.name);
        success=_traj_player->run_trajectory_and_release(t.trajectory);
        break;
    default:
        ROS_ERROR_STREAM("[Move_Maker] Trajectory type unknown!! What kind of trajectory is " << mode << "?");
    }
    if(!success)
    {
        _place_token.setAborted(_place_token_result,traj_id + " trajectory failed");
        return success;
    }
    ros::Duration(INTER_TRAJ_GAP).sleep(); // Let's wait after each trajectory to be sure that the trajectory is done
    return success;
}

Move_Maker::Move_Maker(const char *trajectory_file, const char * service) :
    _place_token(_n, "place_token", boost::bind(&Move_Maker::execute_place_token, this, _1), false)
{
    _prefix_traj="";
    std::vector<trajectory_msgs::JointTrajectory> trajs;
    std::vector<std::string> traj_ids;

    _srv_set_traj_type = _n.advertiseService("set_movement_type", &Move_Maker::set_movement_type,this);

    ROS_ASSERT_MSG(trajectory_xml_parser::read_from_file(trajectory_file,trajs,traj_ids), "Error parsing trajectory xml file.");
    _ttt_traj_n=trajs.size();
    ROS_ASSERT_MSG(_ttt_traj_n==traj_ids.size(),"#trajectories != #trajectory_names");
    ROS_INFO_STREAM("[Move_Maker] " << _ttt_traj_n << " trajectories loaded from xml file");

    _trajectory_repository.reserve(_ttt_traj_n);

    for (size_t i = 0; i < _ttt_traj_n; ++i) {
        _trajectory_repository.push_back(TTT_Trajectory(trajs[i],traj_ids[i]));
    }
    //this->print_trajectory_repository_details();
    ROS_INFO("[Move_Maker] Service is %s",service);

    _traj_player = new Trajectory_Player(service);

    _place_token.start(); //starting the action server
}

bool Move_Maker::make_a_move(std::vector<std::string> traj_names, std::vector<Trajectory_Type> modes)
{
    if(traj_names.size()!=modes.size()) {
        ROS_ERROR_STREAM("[Move_Maker] Move not possible: " << traj_names.size() << " trajs. != " << modes.size() << " types.");
        return false;
    }
    size_t n=modes.size();
    TTT_Trajectory t;
    for (size_t i = 0; i < n; ++i) {
        if(this->get_ttt_trajectory(traj_names[i],t))
        {
            ROS_DEBUG_STREAM("[Move_Maker] Trajectory found. " << t.get_ttt_trajectory_description());
            switch(modes[i])
            {
            case PLAIN:
                ROS_INFO_STREAM("[Move_Maker] Playing PLAIN trajectory " << t.name);
                if(!_traj_player->run_trajectory(t.trajectory)) return false;
                break;
            case GRASP:
                ROS_INFO_STREAM("[Move_Maker] Playing GRASP trajectory " << t.name);
                if(!_traj_player->run_trajectory_and_grasp(t.trajectory)) return false;
                break;
            case RELEASE:
                ROS_INFO_STREAM("[Move_Maker] Playing RELEASE trajectory " << t.name);
                if(!_traj_player->run_trajectory_and_release(t.trajectory)) return false;
                break;
            default:
                ROS_ERROR_STREAM("[Move_Maker] Trajectory type unknown!! What kind of trajectory is " << modes[i] << "?");
                return false;
            }
            ros::Duration(INTER_TRAJ_GAP).sleep(); // Let's wait after each trajectory to be sure that the trajectory is done
        }
        else {
            ROS_WARN_STREAM("[Move_Maker] Trajectory " << traj_names[i] << " not found.");
            return false;
        }
    }

    return true;
}

void Move_Maker::print_trajectory_repository_details()
{
    foreach (TTT_Trajectory t, _trajectory_repository) {
        ROS_INFO_STREAM("[Move_Maker] Trajectory " << t.name << " has " << t.trajectory.points.size() << " points.");
    }
}

bool Move_Maker::set_movement_type(tictactoe::SetTrajectoryType::Request &req, tictactoe::SetTrajectoryType::Response& res)
{
    if (req.smooth) {
        this->set_smooth_trajectories();
        ROS_INFO("[Move_Maker] Using smooth trajectories");
    } else {
        this->set_mechanistic_trajectories();
        ROS_INFO("[Move_Maker] Using mechanistic trajectories");
    }
    res.error=false;
    return true;
}

bool Move_Maker::execute_place_token(const tictactoe::PlaceTokenGoalConstPtr& goal)
{
    ROS_INFO_STREAM("[Move_Maker] Executing the action to place a token to " << goal->cell);
    std::string* traj_ids = this->get_trajectories_to_cell(goal->cell); //It always returns a vector of 3 strings
    _place_token_result.success=false; //initially the action is not successed

    _place_token_feedback.percent_complete=0.0;
    _place_token.publishFeedback(_place_token_feedback);

    // The first trajectory is always to grasp a new token from the heap
    _place_token_result.success=this->execute_single_trajectory(_prefix_traj+traj_ids[0],GRASP);
    _place_token_feedback.percent_complete=0.33;
    _place_token.publishFeedback(_place_token_feedback);
    if (!_place_token_result.success)
    {
        ROS_ERROR_STREAM("[Move_Maker] Movement failed at " << _place_token_feedback.percent_complete*100 << "% of its completion!");
        return false;
    }

    // The second trajectory is used to place the token in the corresponding cell
    _place_token_result.success=this->execute_single_trajectory(_prefix_traj+traj_ids[1],RELEASE);
    _place_token_feedback.percent_complete=0.66;
    _place_token.publishFeedback(_place_token_feedback);
    if (!_place_token_result.success)
    {
        ROS_ERROR_STREAM("[Move_Maker] Movement failed at " << _place_token_feedback.percent_complete*100 << "% of its completion!");
        return false;
    }

    // The third trajectory moves the arm to the initial position
    _place_token_result.success=this->execute_single_trajectory(_prefix_traj+traj_ids[2],PLAIN);
    _place_token_feedback.percent_complete=0.99;
    _place_token.publishFeedback(_place_token_feedback);
    if (!_place_token_result.success)
    {
        ROS_ERROR_STREAM("[Move_Maker] Movement failed at " << _place_token_feedback.percent_complete*100 << "% of its completion!");
        return false;
    }

    _place_token.setSucceeded(_place_token_result);
    return _place_token_result.success;
}

