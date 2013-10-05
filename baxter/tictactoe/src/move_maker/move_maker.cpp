#include "move_maker.h"

namespace ttt
{

bool Move_Maker::get_ttt_trajectory(std::string traj_name, TTT_Trajectory& traj)
{
    if (_trajectory_repository.empty()) {
        ROS_WARN("Empty trajectory repository");
        return false;
    }
    foreach (TTT_Trajectory t, _trajectory_repository) {
        if(t.name==traj_name) traj=t;
        return true;
    }
    return false;
}

Move_Maker::Move_Maker(const char *trajectory_file, const char * service)
{
    std::vector<trajectory_msgs::JointTrajectory> trajs;
    std::vector<std::string> traj_ids;

    ROS_ASSERT_MSG(trajectory_xml_parser::read_from_file(std::string(trajectory_file),trajs,traj_ids), "Error parsing trajectory xml file.");
    ROS_ASSERT_MSG(trajs.size()==traj_ids.size(),"#trajectories != #trajectory_names");
    _ttt_traj_n=trajs.size();
    _trajectory_repository.reserve(_ttt_traj_n);

    for (size_t i = 0; i < _ttt_traj_n; ++i) {
        _trajectory_repository.push_back(TTT_Trajectory(trajs[i],traj_ids[i]));
    }

    _traj_player = new Trajectory_Player(service);
}

bool Move_Maker::make_a_move(std::vector<std::string> traj_names, std::vector<Trajectory_Type> types)
{
    if(traj_names.size()!=types.size()) {
        ROS_ERROR_STREAM("Move not possible: " << traj_names.size() << " trajs. != " << types.size() << " types.");
        return false;
    }
    size_t n=types.size();
    TTT_Trajectory t;
    for (size_t i = 0; i < n; ++i) {
        if(this->get_ttt_trajectory(traj_names[i],t))
        switch(types[i])
        {
        case PLAIN:
            ROS_INFO_STREAM("Playing PLAIN trajectory " << t.name);
            if(!_traj_player->run_trajectory(t.trajectory)) return false;
            break;
        case GRASP:
            ROS_INFO_STREAM("Playing GRASP trajectory " << t.name);
            if(!_traj_player->run_trajectory_and_grasp(t.trajectory)) return false;
            break;
        case RELEASE:
            ROS_INFO_STREAM("Playing RELEASE trajectory " << t.name);
            if(!_traj_player->run_trajectory_and_release(t.trajectory)) return false;
            break;
        default:
            ROS_ERROR_STREAM("Trajectory type unknown!! What kind of trajectory is " << types[i] << "?");
            return false;
        }
        ros::Duration(1.0).sleep(); // Let's wait 1s after each trajectory to be sure that the trajectory is done
    }

    return true;
}

}
